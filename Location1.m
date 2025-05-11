%% Initialize CoppeliaSim Remote API
sim = remApi('remoteApi');
sim.simxFinish(-1); % Close previous connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % Start connection
if clientID > -1
    disp('Connected to CoppeliaSim for path execution.');
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
    [resL, leftMotorHandle] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
    [resR, rightMotorHandle] = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
    [resP, pioneerHandle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx',sim.simx_opmode_blocking);
    if resL ~= sim.simx_return_ok || resR ~= sim.simx_return_ok || resP ~= sim.simx_return_ok
        disp('Error: Unable to get robot or motor handles. Exiting.');
        sim.simxFinish(clientID);
        return;
    end
    %% Get current pose of the robot
    [~, robotPosition] = sim.simxGetObjectPosition(clientID, pioneerHandle, -1,sim.simx_opmode_streaming);
    [~, robotOrientation] = sim.simxGetObjectOrientation(clientID,pioneerHandle, -1, sim.simx_opmode_streaming);
    pause(0.5); % Allow streaming to fetch the latest values
    [~, robotPosition] = sim.simxGetObjectPosition(clientID, pioneerHandle, -1,sim.simx_opmode_buffer);
    [~, robotOrientation] = sim.simxGetObjectOrientation(clientID,pioneerHandle, -1, sim.simx_opmode_buffer);
    % Convert CoppeliaSim position to world coordinates
    robotCurrentPose = double([robotPosition(1), robotPosition(2), robotOrientation(3)]); % [x, y, theta]
    % Load SLAM map
    load('slamMap.mat', 'map');
    % Define goal location
    goalLocation = [-0.4,1.1, pi/2]; % Example target pose

    % Validate goal location
    goalLocationGrid = world2grid(map, goalLocation(1:2));
    if checkOccupancy(map, goalLocationGrid)
        disp('Goal location is invalid. Adjusting...');
        goalLocation(1:2) = findFreeLocation(map, goalLocation(1:2), 0.5);
        goalLocationGrid = world2grid(map, goalLocation(1:2));
        disp(['New Goal Location: ', mat2str(goalLocation(1:2))]);
    end
    %% Path Planning (Delegated to pathPlanner function, which handles visualization)
    path = pathPlanner(robotCurrentPose, goalLocation, map);
    %% Check if a valid path was found
    if isempty(path)
        disp('No valid path found. The robot will not move.');
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
        sim.simxFinish(clientID);
        return;
    end
    %% Path Execution in CoppeliaSim
    executePathPID(clientID, sim, path, pioneerHandle, leftMotorHandle, rightMotorHandle);
    disp('Path execution completed.');
    % Stop simulation
    %sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    %sim.simxFinish(clientID);
    disp('Disconnected from CoppeliaSim.');
else
    disp('Failed to connect to CoppeliaSim.');
end
% Release Remote API resources
%sim.delete();
disp('Remote API resources released.');
%% Helper Function: Find Free Location
function newLocation = findFreeLocation(map, location, stepSize)
theta = 0:pi/8:2*pi;
for r = stepSize:stepSize:2
    x = location(1) + r * cos(theta);
    y = location(2) + r * sin(theta);
    candidates = [x', y'];
    for i = 1:size(candidates, 1)
        if ~checkOccupancy(map, world2grid(map, candidates(i, :)))
            newLocation = candidates(i, :);
            return;
        end
    end
end
disp('Warning: No free location found. Using the original location.');
newLocation = location;
end