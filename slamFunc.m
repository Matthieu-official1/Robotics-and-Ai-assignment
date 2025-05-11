function slamObj = slamFunc(clientID, sim, leftMotorHandle, rightMotorHandle)
    % Initialize lidarSLAM Object
    maxLidarRange = 7; % Maximum range in meters
    resolution = 20; % Map resolution in cells per meter
    slamObj = lidarSLAM(resolution, maxLidarRange);
    slamObj.LoopClosureThreshold = 800; % Set loop closure threshold
    slamObj.LoopClosureSearchRadius = 1; % Set loop closure search radius
    slamObj.MovementThreshold = [0.1, 0.1];

    scanCount = 0; % Initialise a scan counter

    % Initialize teleoperation control variables
    global keyState teleopFig;
    keyState = struct('w', false, 's', false, 'a', false, 'd', false);
    velocity = 2.5; % Speed for forward/backward motion
    turnSpeed = 1.0; % Speed for turning

    % Create figure for teleoperation
    teleopFig = figure('Name', 'Teleoperation Control', ...
        'KeyPressFcn', @keyPress, ...
        'KeyReleaseFcn', @keyRelease, ...
        'CloseRequestFcn', @(src, event) closeTeleop(sim, clientID, leftMotorHandle, rightMotorHandle, slamObj));
    disp('Use W/A/S/D keys to control the robot. Close the figure to stop.');

    % Main control loop
    while isvalid(teleopFig)

        % Update motor velocities based on key states
        [vLeft, vRight] = getMotorVelocities(keyState, velocity, turnSpeed);
        sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, sim.simx_opmode_oneshot );
        sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, sim.simx_opmode_oneshot );

        % Collect and process LIDAR data
        scanCount = processLidarData(sim, clientID, slamObj, scanCount);
   
        pause(0.05);
    end

    % Stop the robot before exiting
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_oneshot );
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_oneshot );
end

% ----------------------------- FUNCTION DEFINITIONS-----------------------------%

%% Key press event handler
function keyPress(~, event)
    global keyState;
    if isfield(keyState, event.Key)
        keyState.(event.Key) = true;
    end
end

%% Key release event handler
function keyRelease(~, event)
    global keyState;
    if isfield(keyState, event.Key)
        keyState.(event.Key) = false;
    end
end

%% Compute motor velocities based on key states
function [vLeft, vRight] = getMotorVelocities(keyState, velocity, turnSpeed)
    vLeft = 0;
    vRight = 0;
    if keyState.w % Move forward
        vLeft = velocity;
        vRight = velocity;
    elseif keyState.s % Move backward
        vLeft = -velocity;
        vRight = -velocity;
    end
    if keyState.a % Turn left
        vLeft = vLeft - turnSpeed;
        vRight = vRight + turnSpeed;
    elseif keyState.d % Turn right
        vLeft = vLeft + turnSpeed;
        vRight = vRight - turnSpeed;
    end
end

%% Process LIDAR data and add to SLAM
function scanCount = processLidarData(sim, clientID, slamObj, scanCount)
    % Try to retrieve LIDAR data
    [rC, hokuyoScanner_PackedData] = sim.simxGetStringSignal(clientID, 'measuredData', sim.simx_opmode_buffer);
    % If retrieval fails or data is empty, exit function immediately
    if rC ~= 0 || isempty(hokuyoScanner_PackedData)
        return;
    end
    % Process valid LIDAR data
    try
        laserScannerData = sim.simxUnpackFloats(hokuyoScanner_PackedData);
        laserScannerData_pt_x = double(laserScannerData(1:3:end));
        laserScannerData_pt_y = double(laserScannerData(2:3:end));
        scanData = lidarScan([laserScannerData_pt_x', laserScannerData_pt_y']);

        % Add scan to SLAM object
        if addScan(slamObj, scanData)
            scanCount = scanCount + 1;
            if mod(scanCount, 10) == 0 % Display message only every 10 scans
                fprintf('Total Scans added: %d \n', scanCount);
            end
        end
    catch ME
        disp(['Error processing LIDAR data: ', ME.message]);
    end
end

%% Cleanup and close teleoperation
function closeTeleop(sim, clientID, leftMotor, rightMotor, slamObj)
    try
        disp('Stopping teleoperation...');
        sim.simxSetJointTargetVelocity(clientID, leftMotor, 0, sim.simx_opmode_oneshot );
        sim.simxSetJointTargetVelocity(clientID, rightMotor, 0, sim.simx_opmode_oneshot );
        % Display and save the SLAM map
        disp('Building maps...');
        fig = figure;
        show(slamObj, 'Poses', 'off');
        hold on;
        show(slamObj.PoseGraph);
        title("Pose Graph with loop closures");
        hold off;
        savefig(fig,"SLAMPoseGraph.fig")
        [scans, optimizedPoses] = scansAndPoses(slamObj);
        map = buildMap(scans, optimizedPoses, slamObj.MapResolution, slamObj.MaxLidarRange);
        fig = figure;
        show(map);
        title("Probabilistic Occupancy Map");
        savefig(fig,"SLAMOccupancyMap.fig")
    catch ME
        disp(['Error during teleop cleanup: ', ME.message]);
    end

    % Close teleoperation figure
    global teleopFig;
    if ishandle(teleopFig)
        delete(teleopFig);
    end
end
