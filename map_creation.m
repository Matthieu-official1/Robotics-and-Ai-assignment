wss% Load the CoppeliaSim Remote API
sim = remApi('remoteApi');
sim.simxFinish(-1); % Close previous connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % Start connection with CoppeliaSim
if clientID == 0 % If connection is successful
    disp('Connected to CoppeliaSim');
    %% Retrieve required object handles
    [~, pioneerHandle] = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/',sim.simx_opmode_blocking);
    [~, leftMotorHandle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [~, rightMotorHandle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
    disp("Successfully retrieved all handles!");
    %% Start the Simulation
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);
    pause(2); % Allow the simulation to initialize
    % Enable LIDAR data streaming
    sim.simxGetStringSignal(clientID, 'measuredData', sim.simx_opmode_streaming);
    %% Call SLAM Mapping Function
    slamObj = slamFunc(clientID, sim, leftMotorHandle, rightMotorHandle); % Runs SLAM-based mapping
    % Extract and process map data
    [scans, optimizedPoses] = scansAndPoses(slamObj);
    map = buildMap(scans, optimizedPoses, slamObj.MapResolution, slamObj.MaxLidarRange);
    save('slamMap.mat', 'map'); % Save occupancy map
    % Save and display the SLAM map
    disp('Building SLAM map...');
    figure;
    show(map);
    title("Probabilistic Occupancy Map");
    savefig("SLAM_OccupancyMap.fig");
    disp('SLAM mapping complete.');
    %% Stop the Simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    disp('Simulation stopped.');
    % Close connection
    sim.simxFinish(clientID);
else
    disp('Failed to connect to CoppeliaSim');
end
% Release Remote API resources
sim.delete();
disp('Remote API resources released.');