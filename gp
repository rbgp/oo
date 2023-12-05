clc
clear
close all
rosshutdown;

% ROS initialization
rosinit("http://192.168.150.130:11311");
odomSub = rossubscriber("/odom", "nav_msgs/Odometry");
laserSub = rossubscriber("/scan"); % Assuming you have a laser scanner topic
robPub = rospublisher("/cmd_vel", "geometry_msgs/Twist", "DataFormat", "struct");
velMsg = rosmessage(robPub);

% Load occupancy map
mapData = load('OccupanyMap1.mat');
map = mapData.myOccMapPro3;

% Plot the initial map
figure;
show(map);
title('Occupancy Map');

% Inflate the map for better visualization
figure;
inflate(map, 0.05);
show(map);
title('Inflated Occupancy Map');

% Robot parameters
robotInitialLocation = [0, 0];
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation, initialOrientation]';

robot = differentialDriveKinematics("TrackWidth", 0.5, "VehicleInputs", "VehicleSpeedHeadingRate");

goalRadius = 1.0;
distanceToGoal = inf;

sampleTime = 100;
linearVelocity = 1; % Adjust the linear velocity as needed
angularVelocity = 1; % Adjust the angular velocity as needed

% Initialize arrays to store robot trajectory
xTrajectory = [];
yTrajectory = [];

% Exploration loop
while distanceToGoal > goalRadius
    % Move the robot forward with increased speed
    velMsg.Linear.X = linearVelocity;
    velMsg.Angular.Z = 0;
    send(robPub, velMsg);
    pause(sampleTime);

    % Stop the robot to evaluate surroundings
    velMsg.Linear.X = 0;
    send(robPub, velMsg);
    pause(sampleTime);

    % Re-compute the distance to the goal
    odomMsg = receive(odomSub);
    pose = odomMsg.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    theta = quat2eul([pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z]);
    robotCurrentPose = [x, y, rad2deg(theta(1))]';
    distanceToGoal = norm(robotCurrentPose(1:2) - robotInitialLocation);

    % Check for obstacles using laser data
    laserData = receive(laserSub);
    ranges = laserData.Ranges;
    if any(ranges < 0.1) % If an obstacle is detected within 0.5 meters
        % Adjust the robot's trajectory to avoid the obstacle
        velMsg.Linear.X = 0;
        velMsg.Angular.Z = angularVelocity;
        send(robPub, velMsg);
        pause(1); % Pause for a short time to adjust the trajectory
    end

    % Store the robot's trajectory
    xTrajectory = [xTrajectory, robotCurrentPose(1)];
    yTrajectory = [yTrajectory, robotCurrentPose(2)];
end

% Stop the robot at the end
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(robPub, velMsg);

% Shutdown ROS
rosshutdown;

% Plot the robot's trajectory
figure;
plot(xTrajectory, yTrajectory, 'LineWidth', 2);
hold on;
plot(robotInitialLocation(1), robotInitialLocation(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('X-axis');
ylabel('Y-axis');
title('Robot Trajectory in the Maze');
legend('Robot Trajectory', 'Start Point');
grid on;
