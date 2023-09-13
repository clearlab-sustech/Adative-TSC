close; clear; clc

load('gait_data.txt');
te=(size(gait_data,1)-1)*0.002;
t = 0:0.002:te;

FL_swing = gait_data(1:1000, 1);
FL_stance = gait_data(1:1000,5);
figure(1);
plot(t(1:1000), FL_swing, t(1:1000), FL_stance);
legend('FL swing time remain', 'FL stance time remain');
axis tight

FR_swing = gait_data(1:2000, 2);
FR_stance = gait_data(1:2000,6);

HL_swing = gait_data(1:2000, 3);
HL_stance = gait_data(1:2000,7);

HR_swing = gait_data(1:2000, 4);
HR_stance = gait_data(1:2000,8);