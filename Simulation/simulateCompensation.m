close all;

%% load gait data

% time, hip, knee, ankle, and toe position data respectively (cm to m)
T = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'B4:B109');
xh = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'E4:F109')/100;
xk = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'G4:H109')/100;
xa = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'K4:L109')/100;
xt = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'Q4:R109')/100;

% velocities
vxh = xlsread('Winter_Appendix_data.xlsx','A2.Filtered_Marker_Kinematics', 'L5:L110');
vyh = xlsread('Winter_Appendix_data.xlsx','A2.Filtered_Marker_Kinematics', 'O5:O110');
vxa = xlsread('Winter_Appendix_data.xlsx','A2.Filtered_Marker_Kinematics', 'AG5:AG110');
vya = xlsread('Winter_Appendix_data.xlsx','A2.Filtered_Marker_Kinematics', 'AJ5:AJ110');

% joint angles (theta, omega, alpha: ankle, knee, hip)
dataq = xlsread('Winter_Appendix_data.xlsx','A4.RelJointAngularKinematics', 'D5:L110');
q = dataq(:,[7 4 1])*pi/180; % degrees to radians (hip, knee, ankle)
qdot = dataq(:,[8 5 2]);
qdotdot = dataq(:,[9 6 3]);

% moments
Ma = xlsread('Winter_Appendix_data.xlsx','A5.ReactionForces&Moments', 'I6:I111');
Mk = xlsread('Winter_Appendix_data.xlsx','A5.ReactionForces&Moments', 'P6:P111');
Mh = xlsread('Winter_Appendix_data.xlsx','A5.ReactionForces&Moments', 'X6:X111');

%% calculations
global L mu_bearing qdotmin g

L_calc = @(i) [norm(xh(i,:)-xk(i,:));
    norm(xk(i,:)-xa(i,:));
    norm(xa(i,:)-xt(i,:))];

L_full = zeros(3,size(xh,1));
for i = 1:size(xh,1)
    L_full(:,i) = L_calc(i);
end

L = mean(L_full,2);
g = 9.81;
m = 1*[1; 0.15];
Izz = m.*([1; 1/8] .* L(1:2)).^2;
Fload = [30; 30];
mu_bearing = 0.0015;
qdotmin = 0.05;
G = 4;
J = pi*(0.0254*2)^4*(0.00238)/4*(2700) * G^2 * ones(2,1);

x = xa-xh;
xfk = zeros(size(q,1),2);
for i = 1:size(xfk,1)
    X = fk(q(i,:)');
    xfk(i,:) = X(:,2)';
end
% Note ankle angle is not strictly accurate for ik.m, it is just the ankle
% angle that makes the foot parallel to the ground
qik = zeros(size(x,1),3);
for i = 1:size(qik,1)
    qp = ik(x(i,:)');
    qik(i,:) = qp';
end

QG = zeros(size(q,1),2);
for i = 1:size(QG,1)
    QG(i,:) = Gcomp(q(i,1:2)',m)';
end

QI = zeros(size(q,1),2);
for i = 1:size(QI,1)
    QI(i,:) = Icomp(q(i,1:2)', qdot(i,1:2)', qdotdot(i,1:2)', m, Izz)';
end

QFR = zeros(size(q,1),2);
for i = 1:size(QFR,1)
    QFR(i,:) = FRcomp(Fload, qdot(i,1:2)')';
end

QID = zeros(size(q,1),2);
for i = 1:size(QID,1)
    QID(i,:) = IDcomp(qdotdot(i,1:2)', J)';
end

Q = QG + QI + QFR + QID;
%% plot
figure(1);
hold on;
scatter(T, QG(:,1), '.k');
scatter(T, QG(:,2), '.r');
xlim([0, 1]);
legend('Hip Joint', 'Knee Joint');
xlabel('Time (s)');
ylabel('Compensation Torque (N*m)');
title('Gravitational Compensation Torque for a 1 Hz Gait')

figure(2);
hold on;
scatter(T, QI(:,1), '.k');
scatter(T, QI(:,2), '.r');
xlim([0, 1]);
legend('Hip Joint', 'Knee Joint');
xlabel('Time (s)');
ylabel('Compensation Torque (N*m)');
title('Inertial Compensation Torque for a 1 Hz Gait')

figure(3);
hold on;
scatter(T, QFR(:,1),50, 'ob');
scatter(T, QFR(:,2),20,'*r');
xlim([0, 1]);
legend('Hip Joint', 'Knee Joint');
xlabel('Time (s)');
ylabel('Compensation Torque (N*m)');
title('Frictional Compensation Torque for a 1 Hz Gait')

figure(4);
hold on;
scatter(T, QID(:,1), '.k');
scatter(T, QID(:,2), '.r');
xlim([0, 1]);
legend('Hip Joint', 'Knee Joint');
xlabel('Time (s)');
ylabel('Compensation Torque (N*m)');
title('Disk Brake Inertial Compensation Torque for a 1 Hz Gait')

figure(5);
hold on;
scatter(T, Q(:,1), '.k');
scatter(T, Q(:,2), '.r');
xlim([0, 1]);
legend('Hip Joint', 'Knee Joint');
xlabel('Time (s)');
ylabel('Compensation Torque (N*m)');
title('Total Compensation Torque for a 1 Hz Gait')