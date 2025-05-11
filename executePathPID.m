function executePath(clientID, sim, path, pioneerHandle, leftMotorHandle, rightMotorHandle)
% Tuned PID Controller Gains (Even Slower)
Kp_linear = 0.015;
Ki_linear = 0.0005;
Kd_linear = 0.025;

Kp_angular = 2.5;
Ki_angular = 0.004;
Kd_angular = 0.07;


% Initialize PID error terms
prev_error_linear = 0;
integral_error_linear = 0;

prev_error_angular = 0;
integral_error_angular = 0;

% First pose retrieval using streaming mode
sim.simxGetObjectPosition(clientID, pioneerHandle, -1, sim.simx_opmode_streaming);
sim.simxGetObjectOrientation(clientID, pioneerHandle, -1, sim.simx_opmode_streaming);
pause(0.2); % Slightly increased pause for smoother updates

% Get initial robot pose
try
    [~, robotPosition] = sim.simxGetObjectPosition(clientID, pioneerHandle, -1, sim.simx_opmode_buffer);
    [~, robotOrientation] = sim.simxGetObjectOrientation(clientID, pioneerHandle, -1, sim.simx_opmode_buffer);
    robotCurrentPose = double([robotPosition(1), robotPosition(2), robotOrientation(3)]);
catch ME
    error("Error retrieving pioneer robot's pose: %s", ME.message);
end

% Initialize movement
goalReached = false;
maxExecutionTime = 60; % Timeout in seconds
startTime = tic; % Start timing execution

while ~goalReached
    % Check timeout
    elapsedTime = toc(startTime);
    % if elapsedTime > maxExecutionTime
    %     disp('Timeout reached: Stopping execution.');
    %     break;
    % end

    % Compute distance and heading error to goal
    goalPosition = path(end, :);
    distanceToGoal = norm(robotCurrentPose(1:2) - goalPosition);
    fprintf("The distance to goal is: %.3f\n", distanceToGoal);
    % If robot is within the goal threshold, stop movement
    if distanceToGoal < 0.02
        goalReached = true;
        break;
    end

    % Compute target heading (desired orientation)
    targetTheta = atan2(goalPosition(2) - robotCurrentPose(2), goalPosition(1) - robotCurrentPose(1));
    angleError = targetTheta - robotCurrentPose(3);

    % Normalize angle error to range [-pi, pi]
    angleError = mod(angleError + pi, 2*pi) - pi;

    % Compute PID for linear velocity
    error_linear = distanceToGoal;
    integral_error_linear = integral_error_linear + error_linear;
    derivative_error_linear = error_linear - prev_error_linear;
    v = Kp_linear * error_linear + Ki_linear * integral_error_linear + Kd_linear * derivative_error_linear;
    prev_error_linear = error_linear;

    % Compute PID for angular velocity
    error_angular = angleError;
    integral_error_angular = integral_error_angular + error_angular;
    derivative_error_angular = error_angular - prev_error_angular;
    omega = Kp_angular * error_angular + Ki_angular * integral_error_angular + Kd_angular * derivative_error_angular;
    prev_error_angular = error_angular;

    % Apply strict speed limits to prevent fast movement
    v = max(min(v, 0.08), -0.08);  % **Max speed is now 0.08 m/s (Very slow)**
    omega = max(min(omega, 0.3), -0.3); % **Reduced turning speed to prevent overshoot**

    % Convert to left and right wheel velocities
    vLeft = (v - omega * 0.331) / 0.09752;
    vRight = (v + omega * 0.331) / 0.09752;
    %rev = -1;

    % Send velocities to the robot
    try
        sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, sim.simx_opmode_oneshot);
        sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, sim.simx_opmode_oneshot);

        %sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, sim.simx_opmode_oneshot);
        %sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, sim.simx_opmode_oneshot);
    catch ME
        error('Error setting motor velocities: %s', ME.message);
    end

    % Get updated robot pose for next iteration
    try
        [~, robotPosition] = sim.simxGetObjectPosition(clientID, pioneerHandle, -1, sim.simx_opmode_buffer);
        [~, robotOrientation] = sim.simxGetObjectOrientation(clientID, pioneerHandle, -1, sim.simx_opmode_buffer);
        robotCurrentPose = double([robotPosition(1), robotPosition(2), robotOrientation(3)]);
    catch ME
        error("Error retrieving pioneer robot's pose: %s", ME.message);
    end

    pause(0.1); % **Increased pause for more stable control**
end

% Stop the robot after reaching the goal or timeout
try
    sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_oneshot);
    pause(1);
catch ME
    error('Error stopping motors after goal reached: %s', ME.message);
end

disp('Robot has reached the goal or timed out.');
end
