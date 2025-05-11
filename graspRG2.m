function graspRG2(clientID, sim, action)
    gripperSignal = 'RG2_open';
    openSignalValue = 1;
    closeSignalValue = 0;
    
    % Check the desired action and send the corresponding action to the gripper
    if strcmp(action, 'open')
        sim.simxSetIntegerSignal(clientID, gripperSignal, openSignalValue, sim.simx_opmode_blocking);
        disp("Signal sent to open the gripper");
    elseif strcmp(action, 'close')
        sim.simxSetIntegerSignal(clientID, gripperSignal, closeSignalValue, sim.simx_opmode_blocking);
        disp("Signal sent to close the gripper");
    else
        error("Unkown Action for the RG2 gripper")
    end
end