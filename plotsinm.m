%% LQR

Desired_interval=2500;
angle_roll = anglevalue(1:Desired_interval,1);
angle_pitch = anglevalue(1:Desired_interval,2);
% angle_yaw = anglevalue(1:Desired_interval,3);

plot(0.01:0.01:25,angle_roll);
hold on
plot(0.01:0.01:25,angle_pitch);
% plot(0.01:0.01:4.75,angle_yaw);
xlim([0,25])
title('LQR control for pitch - Simulation')
xlabel ('Time [s]')
ylabel('Angle [°]')

plot(0.01:0.01:25,zeros(2500,1));
legend('Controlled Roll','Controlled Pitch', 'Reference')
hold off
%% LQI 5 REF PITCH 

Desired_interval=1200;
angle_pitch = anglevalue(1:Desired_interval,2);
plot(0.01:0.01:12,angle_pitch);
xlim([0,12])
title('LQI control for pitch - Simulation')
xlabel ('Time [s]')
ylabel('Angle [°]')
hold on 
plot(0.01:0.01:12,10*ones(1200,1));
legend('Controlled Pitch', 'Reference')

%% LQR FEEDFORWARD

Desired_interval=2500;
% angle_roll = anglevalue(1:Desired_interval,1);
angle_pitch = anglevalue(1:Desired_interval,2);
% angle_yaw = anglevalue(1:Desired_interval,3);

% plot(0.01:0.01:25,angle_roll);
% hold on
plot(0.01:0.01:25,angle_pitch);
hold on
% plot(0.01:0.01:4.75,angle_yaw);
xlim([0,25])
ylim([0,12])
title('LQR Feedforward control for pitch - Simulation')
xlabel ('Time [s]')
ylabel('Angle [°]')

plot(0.01:0.01:25,10*ones(2500,1));
legend('Controlled Pitch', 'Reference')
hold off