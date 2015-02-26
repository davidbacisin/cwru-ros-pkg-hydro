function plotOdom_CmdVel(odom_file, cmd_vel_file)

odom = load(odom_file);

%assemble seconds and nanoseconds to get time stamps
odom_time= odom.data(:,2) + odom.data(:,3)*1e-9;
%subtract off starting time, so starts from t=0
odom_start_time = odom_time(1);
odom_time = odom_time - odom_start_time;

odom_x = odom.data(:,6);
odom_y = odom.data(:,7);
odom_qz = odom.data(:,11);
odom_qw = odom.data(:,12);
odom_heading = 2.0*atan2(odom_qz,odom_qw);%cheap conversion from quaternion to heading for planar motion
odom_vel = odom.data(:,49);
odom_omega = odom.data(:,54);

%load another topic file:
cv = load(cmd_vel_file);

%assemble cmd_vel timestamps
cmd_time = cv.data(:,2) + cv.data(:,3)*1e-9;
cmd_time = cmd_time - odom_start_time; %offset relative to odom time, so time is aligned

cmd_vel = cv.data(:,5);
cmd_omega = cv.data(:,10);

subplot(2,1,1);
plot(cmd_time,cmd_vel,'r', odom_time,odom_vel,'b');
xlabel('time (sec)');
ylabel('speed (meters per sec)');
legend('Command velocity', 'Actual (odom) velocity');
title('Linear velocity: command vs. actual');

subplot(2,1,2);
plot(cmd_time,cmd_omega,'r', odom_time,odom_omega,'b');
xlabel('time (sec)');
ylabel('speed (radians per sec)');
legend('Command omega', 'Actual (odom) omega');
title('Angular velocity: command vs. actual');

end