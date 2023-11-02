function UR3A2test
    clc;

    robot = UR3; % Assuming UR3 is a class or function that initializes your robot
    q0 = [0 0 0 0 0 0];

    axis([-2 2 -2 2 0 2]);
    hold on;

    robot.model.animate(q0);
    robot.model.teach(q0);

    % Define the joint states you want to animate through
    jointStates = {
% % [0 -pi/2 0 0 -pi/2 0] %Start
% [35 -54.7 50.4 -82.8 -90 0]*pi/180 %Mid
% [-51.4 -54.7 72 -104 -90 0]*pi/180 %Above pickup
% [-51.4 -40.3 72 -119 -90 0]*pi/180 %Pickup
% [-51.4 -54.7 72 -104 -90 0]*pi/180 %Above pickup
% [35 -54.7 50.4 -82.8 -90 0]*pi/180 %Mid
% [85.4 -33.1 28.8 -90 -90 0]*pi/180 %Above Drop
% [85.4 -4.3 0 -82.8 -90 0]*pi/180 % Dropoff
% [85.4 -33.1 28.8 -90 -90 0]*pi/180 %Above Drop
%         % Add more joint states as needed


    [0 -90 0 -90 90 0] *pi/180;
    [43 -110 -34 -123 90 0] *pi/180;
    [-22 -120 -36 -111 90 0] *pi/180; 
    [-22 -120 -71 -77 90 0] *pi/180;
    [-22 -120 -36 -111 90 0] *pi/180;
    [43 -110 -34 -123 90 0] *pi/180;
    [90 -143 -4 -121 90 0 ] *pi/180;
    [90 -127 -55 -87 93 0] *pi/180;
    [90 -143 -4 -121 90 0 ] *pi/180;
    [43 -110 -34 -123 90 0] *pi/180;
    };

    for i = 1:length(jointStates)
        qMatrix(i,:) = jointStates{i}; % Use the joint states directly
        robot.model.animate(qMatrix(i,:));
        % pause(0.5); % Pause to observe the motion
    end

%% Initialise ROS Connection
rosinit('192.168.27.1'); % If unsure, please ask a tutor
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

%% Get current Joint State
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
pause(2); % Pause to give time for a message to appear

%% Create and Send Joint angles 
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 10; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = qMatrix(4,:)
% nextJointState_123456 = qMatrix(1,:)
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

%%
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

sendGoal(client,goal);


%%
for i = 1:length(qMatrix)
    currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
    currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
    jointStateSubscriber.LatestMessage

    jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

    [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
    goal.Trajectory.JointNames = jointNames;
    goal.Trajectory.Header.Seq = i;
    goal.Trajectory.Header.Stamp = rostime('Now','system');
    goal.GoalTimeTolerance = rosduration(0.05);
    bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
    durationSeconds = 2; % This is how many seconds the movement will take

    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);     

    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    nextJointState_123456 = qMatrix(i,:)
    % nextJointState_123456 = qMatrix(1,:)
    endJointSend.Positions = nextJointState_123456;
    endJointSend.TimeFromStart = rosduration(durationSeconds);

    goal.Trajectory.Points = [startJointSend; endJointSend];
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

    sendGoal(client,goal);
    disp('Reached goal')
    pause(3);

end