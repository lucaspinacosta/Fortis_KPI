clear all;close all;clc;

%% open all rosbags and parse relevant topics
bag = rosbag('localization_sensors.bag');

%% open all rosbags and parse relevant topics
msgRadioOdom = {};
msgNavsatOdom = {};
msgIMU = {};

bagselectOdom = select(bag,'Topic','/ugv0/telehandler/lio_sam/mapping/odometry');
msgOdom = readMessages(bagselectOdom,'DataFormat','struct');

bagselectGPS = select(bag,'Topic','/ugv0/telehandler/odometry/gps');
msgGPS = readMessages(bagselectGPS,'DataFormat','struct');

bagselectIMU = select(bag,'Topic','/ugv0/telehandler/imu/wit/imu');
msgIMU = readMessages(bagselectIMU,'DataFormat','struct');

%% parse data to arrays to facilitate subsequent maths
for i=1:size(msgOdom, 1)
    tOdom(i,1) = (double(msgOdom{i}.Header.Stamp.Sec) + double(msgOdom{i}.Header.Stamp.Nsec)*10^(-9));
    dOdom(i,1:6) = [msgOdom{i}.Pose.Pose.Position.X, msgOdom{i}.Pose.Pose.Position.Y, msgOdom{i}.Pose.Pose.Orientation.W, msgOdom{i}.Pose.Pose.Orientation.X, msgOdom{i}.Pose.Pose.Orientation.Y, msgOdom{i}.Pose.Pose.Orientation.Z];
end

for i=1:size(msgGPS, 1)
    tGPS(i,1) = (double(msgGPS{i}.Header.Stamp.Sec) + double(msgGPS{i}.Header.Stamp.Nsec)*10^(-9));
    dGPS(i,1:2) = [msgGPS{i}.Pose.Pose.Position.X, msgGPS{i}.Pose.Pose.Position.Y];
end

for i=1:size(msgIMU, 1)
    tIMU(i,1) = (double(msgIMU{i}.Header.Stamp.Sec) + double(msgIMU{i}.Header.Stamp.Nsec)*10^(-9));
    dIMU(i,1:10) = [msgIMU{i}.LinearAcceleration.X, msgIMU{i}.LinearAcceleration.Y, msgIMU{i}.LinearAcceleration.Z, msgIMU{i}.AngularVelocity.X, msgIMU{i}.AngularVelocity.Y, msgIMU{i}.AngularVelocity.Z, msgIMU{i}.Orientation.W, msgIMU{i}.Orientation.X, msgIMU{i}.Orientation.Y, msgIMU{i}.Orientation.Z];
end

%% plot odom and GPS for visualisation purposes
figure(1);
plot(dOdom(:,1),dOdom(:,2),'*');
hold on;
plot(dGPS(:,1),dGPS(:,2),'*');
axis equal

%% calculate KPI02-13: Robot base_link position estimation accuracy within 25 cm and orientation accuracy within 5 degrees
posRMSE = sqrt(mean(sum((dOdom(:,1:2) - dGPS(:,1:2)).^2)));

auxIMU = downsample(dIMU, 30);

eulIMU = quat2eul(auxIMU(:,7:10));
eulOdom = quat2eul(dOdom(:,3:6));
eulRMSE = sqrt(mean((eulOdom(:,3) - eulIMU(:,3)).^2))*180/pi;
