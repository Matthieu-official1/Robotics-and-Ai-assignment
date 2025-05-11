function path = pathPlanner(startLocation, goalLocation, map)
    % Inflate the map slightly to account for robot size
    robotRadius = 0.2; % Adjust based on your robot's size
    inflatedMap = copy(map);
    inflate(inflatedMap, robotRadius);

    % Convert world coordinates (start/goal) to grid coordinates for A*
    % A* planner operates on grid indices
    startGrid = world2grid(inflatedMap, startLocation(1:2));
    goalGrid = world2grid(inflatedMap, goalLocation(1:2));

    % Initialize path as an empty array to prevent undefined variable error
    path = [];

    % Path Planning with A*
    try
        % Initialize the A* planner using the inflated map
        % plannerAStarGrid works directly with occupancyMap objects
        planner = plannerAStarGrid(inflatedMap);

        % Plan the path from start grid coords to goal grid coords
        % The output pathGrid contains grid coordinates [row, col]
        [pathGrid, ~] = plan(planner, startGrid, goalGrid);

        % Check if a valid path was found
        if ~isempty(pathGrid)
            % Convert the grid path back to world coordinates [x, y]
            pathWorld = grid2world(inflatedMap, pathGrid);
            path = pathWorld; % Assign the world coordinate path to the output
            disp("Path successfully found using A*.");
        else
            % A* is deterministic, so retries usually won't help unless parameters change.
            error('No path found using A*. Check map, start/goal locations, and inflation.');
        end
    catch ME
        fprintf('Error during A* path planning: %s\n', ME.message);
        rethrow(ME); % Rethrow the error to stop execution if planning fails
    end

    % If path is still empty here, an error occurred and was thrown.
    % The script would have stopped unless caught higher up.

    % Visualize the path

    %-------------------------- PLOT PATH PLANNINg-------------------------
    figure(1);
    show(inflatedMap);
    hold on;
    % Plot the path using the world coordinates
    pathPlot = plot(path(:, 1), path(:, 2), 'b-', 'LineWidth', 2); % Changed color to blue for distinction
    startPlot = plot(startLocation(1), startLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    goalPlot = plot(goalLocation(1), goalLocation(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    legend([startPlot, goalPlot, pathPlot], {'Start Location', 'Goal Location', 'Planned Path (A*)'}, 'Location', 'northeast'); % Updated legend
    title('Path planned using A*'); % Updated title
    pause(1); % Keep the pause to allow viewing the plot
end