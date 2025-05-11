sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if clientID > -1
    disp('Connected to CoppeliaSim')

    state = "INIT";
    objIndex = 1;
    maxObjects = 3;
    objectList = {'bottle', 'orange', 'cup'};

    while true
        switch state
            case "INIT"
                disp('Initializing...');
                state = "MOVE_TO_LOCATION1";

            case "MOVE_TO_LOCATION1"
                Location1;
                pause(2);
                state = "DETECT_OBJECTS";

            case "DETECT_OBJECTS"
                object_detectionBASE;

                % Store object data
                objectDataMap = containers.Map();
                if exist('bottle', 'var'), objectDataMap('bottle') = bottle; end
                if exist('orange', 'var'), objectDataMap('orange') = orange; end
                if exist('cup', 'var'),    objectDataMap('cup') = cup;    end

                pause(2);
                state = "GRASP_OBJECT";

            case "GRASP_OBJECT"
                currentLabel = objectList{objIndex};
                disp(['Grasping object: ', currentLabel]);

                if isKey(objectDataMap, currentLabel)
                    objectData = objectDataMap(currentLabel);

                    Cartesian_grasping_script1(currentLabel, objectData);
                    pause(2);
                    state = "TRANSIT";
                else
                    disp(['No data found for object: ', currentLabel]);
                    state = "INCREMENT";
                end

            case "TRANSIT"
                Transist;
                pause(2);
                state = "MOVE_TO_LOCATION2";

            case "MOVE_TO_LOCATION2"
                Location2(currentLabel);  % Pass current object label
                pause(4);
                Drop;
                pause(2);
                state = "RELEASE_OBJECT";


            case "RELEASE_OBJECT"
                graspRG2(clientID, sim, 'open');
                pause(4);
                Origin;
                pause(2);
                state = "INCREMENT";

            case "INCREMENT"
                objIndex = objIndex + 1;
                reverse = -1;
                sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, reverse, sim.simx_opmode_oneshot);
                sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, reverse, sim.simx_opmode_oneshot);
                pause(6);
                sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_oneshot);
                sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_oneshot);
                pause(1);
                if objIndex > maxObjects
                    state = "FINISH";
                else
                    state = "MOVE_TO_LOCATION1";
                end

            case "FINISH"
                disp('Finished processing all objects.');
                break;
        end
    end

else
    disp('Failed to connect to CoppeliaSim');
end
