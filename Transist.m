
disp('Program started');

%% Initialization
dof = 6;              % Degrees of freedom for UR3
d = zeros(1, dof+1);  % Link offsets
a = zeros(1, dof);    % Link lengths

% Connect to CoppeliaSim
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

% Define DH Parameters for UR3 (update these values as per your robot's specification)
d(1) = 0.10887;   d(2) = 0.11154;  d(3) = 0;       d(4) = 0;
d(5) = 0.11223 - 0.11154;  d(6) = 0.194 - 0.11223;  d(7) = 0;
a(2) = 0.35252 - 0.10887;  a(3) = 0.56577 - 0.35252;
a(4) = 0.64999 - 0.56577;  a(5) = 0.65111 - 0.64999;
joint_handle = 'UR3_joint';
fprintf('X Y Z: %.3f %.3f %.3f\n', rframex, rframey, rframez);
%% Define Cartesian Waypoints for Grasping Task
% Each row: [x, y, z, roll, pitch, yaw] (position in meters, angles in degrees)
% For example, the waypoints may include approaching the object, going over it, and then post-grasp.

%priorityList = {'orange', 'bottle', 'cup'};
priorityList = {'bottle', 'orange', 'cup'};
% priorityList = {'cup', 'bottle', 'orange'};

for i = 1:length(priorityList)
    label = priorityList{i};
    switch label
        case 'orange'
            if ~isempty(orange)
                rframex = str2double(orange(2));
                rframey = str2double(orange(3));
                rframez = str2double(orange(4));

                break;
            end
        case 'bottle'
            if ~isempty(bottle)
                rframex = str2double(bottle(2));
                rframey = str2double(bottle(3));
                rframez = str2double(bottle(4));
                break;
            end
        case 'cup'
            if ~isempty(cup)
                rframex = str2double(cup(2));
                rframey = str2double(cup(3));
                rframez = str2double(cup(4));
                break;
            end
    end
end

waypoints = [
    -rframex, -rframey, rframez + 0.4, 0, 180, 0;
    -rframex, -rframey , rframez + 0.6, 0,180, 0;
    -rframex, -rframey-0.2 , rframez + 0.9, 0,180, 0;

    ];
numWaypoints = size(waypoints, 1);

if clientID > -1
    disp('Connected to CoppeliaSim');

    %% Retrieve Joint Handles
    jh = zeros(1, dof);
    for i = 1:dof
        [~, jh(i)] = sim.simxGetObjectHandle(clientID, strcat(joint_handle, int2str(i)), sim.simx_opmode_blocking);
    end

    %% Retrieve Current Joint Positions
    currentJoints = zeros(1, dof);
    for i = 1:dof
        [~, currentJoints(i)] = sim.simxGetJointPosition(clientID, jh(i), sim.simx_opmode_blocking);
    end
    disp('Current Joint Positions (radians):');
    disp(currentJoints);

    %% Compute IK Solutions for Each Waypoint
    % Preallocate a matrix to store the joint configurations for each waypoint
    targetJointsAll = zeros(numWaypoints, dof);

    for wp = 1:numWaypoints
        % Extract the waypoint's position and orientation
        pos = waypoints(wp, 1:3)';  % Convert row to column vector
        ori = waypoints(wp, 4:6);   % [roll, pitch, yaw] in degrees

        % Convert the orientation to a rotation matrix
        R_target = rpy2rotm(deg2rad(ori));
        % Create the 4x4 homogeneous transformation matrix
        T_target = [R_target, pos; 0 0 0 1];

        % Compute the inverse kinematics solution(s) for this pose
        joints = invKin8sol(d, a, T_target);
        % Select one solution (e.g., the first solution) for this waypoint
        targetJointsAll(wp, :) = joints(4, :);

        fprintf('Waypoint %d target joints (radians):\n', wp);
        disp(targetJointsAll(wp, :));
    end

    %% Create Full Joint Configuration Sequence
    % Prepend the current joint configuration so that the trajectory goes from current position to waypoint 1, then to waypoint 2, etc.
    jointConfigSeq = [currentJoints; targetJointsAll];
    numSegments = size(jointConfigSeq, 1) - 1;

    %% Plan and Execute Trajectory Segments Between Waypoints
    for seg = 1:numSegments
        % Define trajectory parameters for this segment
        T_seg = 5;                  % Duration of the segment in seconds
        timeStep = 0.05;            % Time step for trajectory execution
        timeVec = 0:timeStep:T_seg; % Time vector

        % Preallocate trajectory matrix for the current segment
        trajSegment = zeros(length(timeVec), dof);

        % Generate a quintic trajectory for each joint for the segment
        for j = 1:dof
            trajSegment(:, j) = quinticTrajectory(timeVec, T_seg, ...
                jointConfigSeq(seg, j), jointConfigSeq(seg+1, j), ...
                0, 0, 0, 0);
        end

        % (Optional) Plot the joint trajectories for the current segment
        %---------------------------------------------- figure;
        % for j = 1:dof
        %     subplot(3,2,j);
        %     plot(timeVec, trajSegment(:, j), 'LineWidth', 1.5);
        %     xlabel('Time (s)');
        %     ylabel(sprintf('Joint %d (rad)', j));
        %     title(sprintf('Segment %d - Joint %d', seg, j));
        %     grid on;
        % end
        % sgtitle(sprintf('Joint Trajectories for Segment %d', seg));

        % Execute the trajectory segment on the UR3 in simulation
        disp(['Executing trajectory segment ', num2str(seg), ' of ', num2str(numSegments)]);
        for idx = 1:length(timeVec)
            for j = 1:dof
                sim.simxSetJointTargetPosition(clientID, jh(j), trajSegment(idx, j), sim.simx_opmode_oneshot);
            end
            pause(timeStep);
        end
    end
    %% Disconnect from CoppeliaSim
    reverse = -1;
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, reverse, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, reverse, sim.simx_opmode_oneshot);
    pause(6);
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_oneshot);
    pause(1);
    sim.simxFinish(clientID);
    sim.delete();
    disp('Disconnected from CoppeliaSim');
else
    disp('Failed to connect to CoppeliaSim');
end

%% Quintic Trajectory Function
function theta = quinticTrajectory(t, T, theta0, thetaf, vel0, velf, acc0, accf)
% Constructs a quintic polynomial trajectory for smooth motion
A = [1,    0,      0,       0,         0,          0;
    1,    T,      T^2,     T^3,       T^4,        T^5;
    0,    1,      0,       0,         0,          0;
    0,    1,      2*T,     3*T^2,     4*T^3,      5*T^4;
    0,    0,      2,       0,         0,          0;
    0,    0,      2,       6*T,       12*T^2,     20*T^3];
b = [theta0; thetaf; vel0; velf; acc0; accf];
coeffs = A\b;
theta = coeffs(1) + coeffs(2)*t + coeffs(3)*t.^2 + coeffs(4)*t.^3 + coeffs(5)*t.^4 + coeffs(6)*t.^5;
end

%% rpy2rotm Function
function R = rpy2rotm(rpy)
% Converts roll, pitch, and yaw (in radians) to a rotation matrix using the ZYX sequence.
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
R_x = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
R_y = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
R_z = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
R = R_z * R_y * R_x;  % ZYX rotation sequence
end
