% Initialize the CoppeliaSim remote API
sim = remApi('remoteApi');
sim.simxFinish(-1); % Close any existing connections
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
if clientID > -1
    disp('Connected to CoppeliaSim');
    % Get handles for the vision sensors
    [~, perspHandle] = sim.simxGetObjectHandle(clientID, 'Vision_sensor_persp', sim.simx_opmode_blocking);
    [~, orthoHandle] = sim.simxGetObjectHandle(clientID, 'Vision_sensor_ortho', sim.simx_opmode_blocking);
    disp('Vision sensor handles retrieved successfully.');
    % Get handle for the UR3 robot
    [~, ur3Handle] = sim.simxGetObjectHandle(clientID, 'UR3', sim.simx_opmode_blocking);
    disp('Initializing YOLO object detector...');
    model = yolov4ObjectDetector('csp-darknet53-coco'); % Pre-trained YOLOv4 model
    % Main loop: Process 6
    for frame = 1:6
        % Capture images from the perspective and orthogonal sensors
        [res1, resolution1, image1] = sim.simxGetVisionSensorImage2(clientID, perspHandle, 0, sim.simx_opmode_blocking);
        % Check if images were successfully captured
        if res1 == sim.simx_return_ok
            % Process Perspective Sensor Image
            disp('Processing image from Perspective Sensor...');
            [bboxes1, scores1, labels1] = detect(model, image1);
            detectedImage1 = insertObjectAnnotation(image1, 'rectangle', bboxes1, labels1, 'Color', 'yellow', 'TextBoxOpacity', 0.7, 'FontSize', 12);
            % Estimate object poses for both images
            objectPoses1 = estimateObjectPose(sim, clientID, perspHandle, model);
            % Retrieve UR3's position and orientation in the simulation
            [~, UR3Position] = sim.simxGetObjectPosition(clientID, ur3Handle, -1, sim.simx_opmode_blocking);
            [~, UR3Orientation] = sim.simxGetObjectOrientation(clientID, ur3Handle, -1, sim.simx_opmode_blocking);
            % Convert object positions to the UR3 robot's coordinate frame
            orange = [];
            bottle = [];
            cup = [];
            for i = 1:length(objectPoses1)
                tag = objectPoses1(i).Label;
                Lpos = objectPoses1(i).Position;

                % Convert from global position to UR3 robot's frame
                baseFramePos = WorldFrametoUr3(objectPoses1(i).Position, UR3Position, UR3Orientation);
                rframex = baseFramePos(1);
                rframey = baseFramePos(2);
                rframez = baseFramePos(3);
                fprintf('%s WFrame X: %.3f Y: %.3f Z: %.3f\n',  tag, objectPoses1(i).Position(1), objectPoses1(i).Position(2), objectPoses1(i).Position(3));
                fprintf('%s RFrame X: %.3f Y: %.3f Z: %.3f\n',  tag, rframex, rframey, rframez );

                label = objectPoses1(i).Label;
                position = objectPoses1(i).Position;
                if ~isempty(label)
                    labelStr = string(label);
                    data = [labelStr, rframex, rframey, rframez];

                    switch label
                        case "orange"
                            orange = data;
                            disp(orange);
                        case "bottle"
                            bottle = data;
                            disp(bottle);
                        case "cup"
                            cup = data;
                            disp(cup);
                        otherwise
                            % No action for other labels
                    end
                else
                    disp('no object value stored')
                end
            end
            % Display the annotated images
            figure(2);
            imshow(detectedImage1);
            title('Perspective Sensor Detection');
            pause(0.5); % Pause to display the results
        else
            disp('Failed to capture image from one or both sensors.');
        end
    end

    sim.simxFinish(clientID);
    disp('Disconnected from CoppeliaSim');
else
    disp('Failed to connect to CoppeliaSim');
end
sim.delete(); % Clear API instance

% Function to estimate object pose using the YOLO detector
function objectPoses = estimateObjectPose(sim, clientID, visionSensorHandle, model)
objectPoses = [];
% Capture image from the vision sensor
[res, ~, capturedImage] = sim.simxGetVisionSensorImage2(clientID, visionSensorHandle, 0, sim.simx_opmode_blocking);
if res == sim.simx_return_ok
    % Perform object detection
    [bboxes,scores, labels] = detect(model, capturedImage);
    if ~isempty(bboxes)
        disp('Objects detected:');
        disp(labels);
        % Get the vision sensor's pose in the simulation world
        [~, visionSensorPose] = sim.simxGetObjectPosition(clientID, visionSensorHandle, -1, sim.simx_opmode_blocking);
        [~, visionSensorOrientation] = sim.simxGetObjectOrientation(clientID, visionSensorHandle, -1, sim.simx_opmode_blocking);
        % Convert bounding box centers to 3D positions
        for i = 1:size(bboxes, 1)
            % Extract bounding box center (2D pixel coordinates)
            bbox = bboxes(i, :);
            bboxCenterX = bbox(1) + bbox(3) / 2;
            bboxCenterY = bbox(2) + bbox(4) / 2;
            % Normalize to sensor image space (-1 to 1 range)
            normX = -(bboxCenterX - size(capturedImage, 2) / 2) /(size(capturedImage, 2) / 2);
            normY = (size(capturedImage, 1) / 2 - bboxCenterY) / (size(capturedImage, 1) / 2);
            % Use depth and camera intrinsics to estimate 3D position
            % Assume an approximate depth (can be retrieved from depth sensor if available)
            approxDepth = 0.511; % Adjust this based on the real scene

            x = normX * approxDepth * 0.52; % Scale normalized x by depth
            y = normY * approxDepth; % Scale normalized y by depth
            z = approxDepth;
            % Transform to world coordinates using vision sensor pose
            objectWorldPosition = transformToWorldFrame([x, y, z], visionSensorPose, visionSensorOrientation);
            % Store object pose
            objectPoses(end+1).Label = labels(i); % Object label
            objectPoses(end).Position = objectWorldPosition; % 3D position
            objectPoses(end).Orientation = visionSensorOrientation; % Orientation (assume same as sensor for simplicity)
        end
    else
        disp('No objects detected.');
    end
else
    disp('Failed to capture image from the vision sensor.');
end
end

% Function to convert from local position (sensor frame) to world frame
function worldPosition = transformToWorldFrame(localPosition, sensorPosition, sensorOrientation)
% Convert a 3D local position in the vision sensor frame to the world frame.
% Create rotation matrix from sensor orientation
rotationMatrix = eul2rotm(sensorOrientation, 'XYZ');
% Transform local position to world position
worldPosition = (rotationMatrix * localPosition')' + sensorPosition;
end

% Function to transform global position to the UR3 robot's frame
% function UR3CPosition = WorldFrametoUr3(sim,clientID,globalPosition, UR3Position, UR3Orientation)
% % Convert a 3D global position from the world frame to the UR3 robot's frame.
% % Create rotation matrix from UR3 orientation
% rotationMatrix = inv(eul2rotm(UR3Orientation, 'XYZ'));
% % Transform global position to UR3's coordinate frame
% UR3relv = (rotationMatrix * UR3Position')';
% UR3CPosition = (rotationMatrix * globalPosition')' - UR3relv;
%
% end
function baseFramePos = WorldFrametoUr3(globalPosition, UR3Position, UR3Orientation)
% disp('Object Position in World Frame:');
% disp(worldPosition);
% baseOri is [roll, pitch, yaw] in radians
R = eul2rotm(UR3Orientation, 'XYZ');    % Base orientation (world → base rotation)
T = [R, UR3Position(:); 0 0 0 1];    % Homogeneous transformation from base to world
T_inv = inv(T);                  % Invert to get world to base transformation
worldHom = [globalPosition(:); 1];  % Convert to homogeneous [x; y; z; 1]
baseHom = T_inv * worldHom;        % Transform world → base
baseFramePos = baseHom(1:3);       % Extract XYZ in base frame
end
