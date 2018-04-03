%% simulateResistance.m
% Uses Winter's gait cycle data to test the gait correction algorithm in
% changeState.m, torque.m, and resistance.m

close all;
clearvars;

%% load gait data

% time, hip, knee, ankle, and toe position data respectively (cm to m)
T = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'B4:B109');
xh = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'E4:F109')/100;
xk = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'G4:H109')/100;
xa = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'K4:L109')/100;
xt = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'Q4:R109')/100;

% load test data
load('simulateResistancePos');
load('simulateResistanceAngles');
load('simulateResistanceVel');
X = current;
q = angles;
v = [0 0; vel];

%% initialize
global X1 X2 nHyst K d L

L_calc = @(i) [norm(xh(i,:)-xk(i,:));
    norm(xk(i,:)-xa(i,:));
    norm(xa(i,:)-xt(i,:))];
L = L_calc(1);

L_full = zeros(3,size(xh,1));
for i = 1:size(xh,1)
    L_full(:,i) = L_calc(i);
end

N = 106;

L = mean(L_full,2);

a0 = 64;
a1 = 99;
a2 = 22;
a3 = 71;
nHyst = 3;
K = 100;
d = 0.01;
state0 = 0;

%% calculate
x = xa-xh;

X1 = x(a0:a1,:);
X2 = x(a2:a3,:);

q = zeros(size(x));

for i = 1:N
    qi = ik(x(i,:));
    q(i,:) = qi(1:2);
end

%%
figure(1);
subplot(1,2,1);
hold on;
subplot(1,2,2);
hold on;
for i = 1:N
    subplot(1,2,1);
    scatter(q(i,1), q(i,2), 'r');
    
    subplot(1,2,2);
    scatter(x(i,1), x(i,2), 'k');
    pause(1);
end

