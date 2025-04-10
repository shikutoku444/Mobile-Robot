% Fully Enhanced Medical Assistant TurtleBot Simulation with Live Navigation
clc; clear; close all;

%% ==================== 1. Environment Setup ====================
map = binaryOccupancyMap(20, 20, 5);
% Add more static obstacles (hospital-like environment)
obstaclePositions = [
    5 5; 5 6; 5 7; 5 8;  % Wall section
    10 10; 10 11; 10 12;  % Another wall
    13 14; 14 14; 15 14;  % Bed or equipment
    6 6; 7 6; 8 6;        % Cabinet
    15 5; 16 5; 17 5;     % Additional obstacles
    8 15; 9 15; 10 15;    % More obstacles
    12 8; 12 9; 12 10     % Vertical wall
];
setOccupancy(map, obstaclePositions, 1);

% Dynamic obstacles: simulated moving people
dynamicObstacles = [8 8; 12 10; 15 15; 4 12; 16 8];

%% ==================== 2. Robot Configuration ====================
robotRadius = 0.3;
batteryCapacity = 100; % Battery capacity in percentage
energyConsumptionRate = 0.1; % Energy consumed per meter traveled

% Generate dynamic start and goal positions
function pos = findFreePosition(map)
    while true
        pos = [randi([1, map.XWorldLimits(2)-1]), randi([1, map.YWorldLimits(2)-1])];
        if ~checkOccupancy(map, pos)
            break;
        end
    end
end

start = findFreePosition(map);
goal = findFreePosition(map);

% Ensure minimum separation between start and goal
while norm(start - goal) < 5
    goal = findFreePosition(map);
end

pose = [start 0];  % [x, y, theta]
robot = differentialDriveKinematics(...
    "TrackWidth", 0.5, ...
    "VehicleInputs", "VehicleSpeedHeadingRate", ...
    "WheelRadius", 0.1, ...
    "WheelSpeedRange", [-10 10]*2*pi);

%% ==================== 3. Create Visualization Figures ====================
fig1 = figure('Name', 'TurtleBot Navigation', 'Position', [100 100 800 700]);
ax1 = axes(fig1);
show(map, 'Parent', ax1);
hold(ax1, 'on');
grid(ax1, 'on');
ax1.GridAlpha = 0.3;

% Initialize start and goal markers
startPlot = plot(ax1, start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
goalPlot = plot(ax1, goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
title(ax1, 'Live TurtleBot Navigation');
xlabel(ax1, 'X Coordinate (meters)'); 
ylabel(ax1, 'Y Coordinate (meters)');

% LIDAR View Figure
fig2 = figure('Name', 'LIDAR View', 'Position', [900 100 800 700]);
lidarFig = polaraxes;
rlim(lidarFig, [0 5]);
title(lidarFig, 'Live LIDAR Scan');
grid(lidarFig, 'on');

%% ==================== 4. Path Planning Configuration ====================
ss = stateSpaceSE2([map.XWorldLimits; map.YWorldLimits; [-pi pi]]);
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 1;
planner = plannerRRTStar(ss, sv);
planner.MaxIterations = 3000;
planner.MaxConnectionDistance = 2;
planner.GoalBias = 0.3;

controller = controllerPurePursuit;
controller.DesiredLinearVelocity = 0.4;
controller.MaxAngularVelocity = 2.0;
controller.LookaheadDistance = 0.5;

%% ==================== 5. Simulation Parameters ====================
goalRadius = 0.5;
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
global simulationRunning;
simulationRunning = true;

%% ==================== 6. Initialize Live Visualization ====================
robotBody = plot(ax1, pose(1), pose(2), 'ks', 'MarkerSize', 12, 'LineWidth', 2);
robotOrientation = quiver(ax1, pose(1), pose(2), 0.5*cos(pose(3)), 0.5*sin(pose(3)), 'r', 'LineWidth', 1.5);
traveledPath = pose(1:2);
travelPlot = plot(ax1, traveledPath(:,1), traveledPath(:,2), 'b-', 'LineWidth', 1.5);

%% ==================== Data Collection for Graph ====================
timeData = [];
distanceData = [];
batteryLevel = batteryCapacity;

%% ==================== 7. Main Simulation Loop ====================
simulationTime = 0;
start_pose = [start 0];
goal_pose = [goal 0];
[pthObj, solnInfo] = plan(planner, start_pose, goal_pose);

if ~solnInfo.IsPathFound
    path = [linspace(start(1), goal(1), 100)', linspace(start(2), goal(2), 100)', zeros(100, 1)];
    warning('Using straight-line fallback path');
else
    path = pthObj.States;
end

interp_factor = 10;
xi = interp1(1:size(path, 1), path(:, 1), linspace(1, size(path, 1), size(path, 1) * interp_factor))';
yi = interp1(1:size(path, 1), path(:, 2), linspace(1, size(path, 1), size(path, 1) * interp_factor))';
interp_path = [xi yi];

pathPlot = plot(ax1, path(:, 1), path(:, 2), 'g-', 'LineWidth', 2);
release(controller);
controller.Waypoints = path(:, 1:2);

while simulationRunning
    if isempty(interp_path)
        disp("No valid path found!");
        break;
    end

    for i = 1:size(interp_path, 1)
        if ~simulationRunning
            disp('Simulation stopped by user');
            break;
        end

        % Update dynamic obstacles
        for j = 1:size(dynamicObstacles, 1)
            angle = rand() * 2 * pi;
            dist = 0.1 + rand() * 0.1;
            dynamicObstacles(j, :) = dynamicObstacles(j, :) + dist * [cos(angle) sin(angle)];
            dynamicObstacles(j, :) = max(min(dynamicObstacles(j, :), 19.5), 0.5);
            setOccupancy(map, round(dynamicObstacles(j, :)), 1);
        end

        % Update position and velocity
        newPos = interp_path(i, :);
        velocity = (newPos - pose(1:2)) / sampleTime;
        pose(1:2) = newPos;
        traveledPath = [traveledPath; pose(1:2)];
        simulationTime = simulationTime + sampleTime;

        % Collect data for graph
        distance = norm(pose(1:2) - start);
        timeData = [timeData; simulationTime];
        distanceData = [distanceData; distance];

        % Update battery level
        batteryLevel = batteryLevel - energyConsumptionRate * norm(velocity) * sampleTime;
        if batteryLevel <= 10
            disp('Low battery! Returning to charging station...');
            goal = start; % Return to start position
            simulationRunning = false;
            break;
        end

        % Update robot visualization
        set(robotBody, 'XData', pose(1), 'YData', pose(2));
        set(robotOrientation, 'XData', pose(1), 'YData', pose(2), ...
            'UData', 0.5 * cos(pose(3)), 'VData', 0.5 * sin(pose(3)));
        set(travelPlot, 'XData', traveledPath(:, 1), 'YData', traveledPath(:, 2));

        % Update LIDAR view
        [ranges, angles] = simulateLidar(pose, map, dynamicObstacles);
        figure(fig2);
        cla(lidarFig);
        polarplot(lidarFig, angles, ranges, 'b.');
        hold(lidarFig, 'on');
        closeIdx = ranges < 1.0;
        if any(closeIdx)
            polarplot(lidarFig, angles(closeIdx), ranges(closeIdx), 'r.');
        end
        polarplot(lidarFig, [0 0], [0 0.5], 'r-', 'LineWidth', 2);
        title(lidarFig, 'Live LIDAR Scan (5m range)');
        rlim(lidarFig, [0 5]);
        hold(lidarFig, 'off');

        % Check if reached goal
        if norm(pose(1:2) - goal) < goalRadius
            disp("Goal Reached!");
            simulationRunning = false; % Stop the simulation
            break;
        end

        % Check for collisions and replan if necessary
        needsReplan = false;

        % Check for dynamic obstacles
        for j = 1:size(dynamicObstacles, 1)
            if norm(pose(1:2) - dynamicObstacles(j, :)) < (robotRadius + 0.3)
                disp("Dynamic obstacle detected! Replanning...");
                needsReplan = true;
                break;
            end
        end

        % Check for static obstacles
        [isOccupied, ~] = getOccupancy(map, pose(1:2));
        if isOccupied
            disp("Static obstacle collision! Replanning...");
            needsReplan = true;
        end

        if needsReplan
            % Clear temporary dynamic obstacles from the map
            for j = 1:size(dynamicObstacles, 1)
                setOccupancy(map, round(dynamicObstacles(j, :)), 0);
            end

            % Recalculate the path
            start_pose = [pose(1:2) 0];
            [pthObj, solnInfo] = plan(planner, start_pose, goal_pose);
            if ~solnInfo.IsPathFound
                path = [linspace(pose(1), goal(1), 100)', linspace(pose(2), goal(2), 100)', zeros(100, 1)];
                warning('Using straight-line fallback path');
            else
                path = pthObj.States;
            end

            % Interpolate the new path for smoother animation
            xi = interp1(1:size(path, 1), path(:, 1), linspace(1, size(path, 1), size(path, 1) * interp_factor))';
            yi = interp1(1:size(path, 1), path(:, 2), linspace(1, size(path, 1), size(path, 1) * interp_factor))';
            interp_path = [xi yi];

            % Update controller waypoints
            release(controller);
            controller.Waypoints = path(:, 1:2);

            % Visualize the new path
            delete(pathPlot);
            pathPlot = plot(ax1, path(:, 1), path(:, 2), 'g-', 'LineWidth', 2);
        end

        waitfor(vizRate);
    end

    % Clear temporary dynamic obstacles from the map
    for j = 1:size(dynamicObstacles, 1)
        setOccupancy(map, round(dynamicObstacles(j, :)), 0);
    end
end

%% ==================== 8. Finalize Visualization ====================
figure(fig1);
legend([startPlot, goalPlot, robotBody, pathPlot, travelPlot], ...
    {'Start', 'Goal', 'Robot', 'Planned Path', 'Traveled Path'}, ...
    'Location', 'northeastoutside');
figure(fig2);
text(10, 10, 'MISSION COMPLETE', 'Color', 'g', 'FontSize', 14, 'FontWeight', 'bold');

%% ==================== Plot Distance vs Time Graph ====================
figure;
plot(timeData, distanceData, 'b-', 'LineWidth', 1.5);
title('Distance Traveled Over Time');
xlabel('Time (s)');
ylabel('Distance Traveled (m)');
grid on;

%% ==================== Helper Functions ====================
function [ranges, angles] = simulateLidar(pose, map, dynamicObstacles)
    angles = linspace(-pi, pi, 180);
    ranges = zeros(size(angles));
    maxRange = 5;
    for i = 1:length(angles)
        staticRange = customRaycast(map, pose, angles(i), maxRange);
        dynamicRange = maxRange;
        for j = 1:size(dynamicObstacles, 1)
            [intersect, dist] = lineCircleIntersection(...
                pose(1:2), angles(i), dynamicObstacles(j, :), 0.3);
            if intersect && dist < dynamicRange
                dynamicRange = dist;
            end
        end
        ranges(i) = min(staticRange, dynamicRange);
    end
end

function [intersect, dist] = lineCircleIntersection(origin, angle, center, radius)
    dx = cos(angle);
    dy = sin(angle);
    f = center - origin;
    a = dx^2 + dy^2;
    b = 2 * (f(1) * dx + f(2) * dy);
    c = f(1)^2 + f(2)^2 - radius^2;
    discriminant = b^2 - 4 * a * c;
    if discriminant < 0
        intersect = false;
        dist = Inf;
    else
        discriminant = sqrt(discriminant);
        t1 = (-b - discriminant) / (2 * a);
        t2 = (-b + discriminant) / (2 * a);
        if t1 >= 0 || t2 >= 0
            intersect = true;
            dist = min(t1, t2);
            if dist < 0, dist = max(t1, t2); end
        else
            intersect = false;
            dist = Inf;
        end
    end
end

function range = customRaycast(map, pose, angle, maxRange)
    x = pose(1); y = pose(2); theta = pose(3) + angle;
    stepSize = 0.05; steps = maxRange / stepSize;
    range = maxRange;
    for k = 1:steps
        x = x + stepSize * cos(theta);
        y = y + stepSize * sin(theta);
        if x < 0 || x > map.GridSize(1) / map.Resolution || ...
           y < 0 || y > map.GridSize(2) / map.Resolution
            range = (k - 1) * stepSize; break;
        end
        [isOccupied, ~] = getOccupancy(map, [x y]);
        if isOccupied, range = k * stepSize; break; end
    end
end
