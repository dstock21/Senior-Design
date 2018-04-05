%% simulate.m
% Uses Winter's gait cycle data to simulate the kinematics of a gait cycle
% and test kinematic functions

close all;
clearvars;

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
global L

L_calc = @(i) [norm(xh(i,:)-xk(i,:));
    norm(xk(i,:)-xa(i,:));
    norm(xa(i,:)-xt(i,:))];
L = L_calc(1);

L_full = zeros(3,size(xh,1));
for i = 1:size(xh,1)
    L_full(:,i) = L_calc(i);
end

L = mean(L_full,2);

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

Q = zeros(size(q,1),2);
for i = 1:size(Q,1)
    % UPDATE with force
    %Q(i,2) = torque(q(i,:)',F);
end

%% refines and interpolates data
r = 22:length(x)-15;
x = x(r,:);
q = q(r,:);
x_new = x(1,:);
q_new = q(1,:);
for i = 2:length(r)
    x_new = [x_new; (x(i-1,:)+x(i,:))/2; x(i,:)];
    q_new = [q_new; (q(i-1,:)+q(i,:))/2; q(i,:)];
end
x = x_new;
q = q_new;

%%

N = 100;
[X, Y] = meshgrid(linspace(-40,60,N), linspace(-10,80,N));
x = zeros(N,N);
iq = zeros(N,N);
q = q*180/pi;

K = 0.1;
ck = zeros(size(x));
ch = zeros(size(x));

for i = 1:N
    for j = 1:N
        err = inf;
        for k = 1:length(q)
            temp = (X(j,i)-q(k,1))^2 + (Y(j,i) - q(k,2))^2;
            if temp < err
                err = temp;
                x(j,i) = err;
                iq(j,i) = k;
                ch(j,i) = abs(X(j,i)-q(k,1));
                ck(j,i) = abs(Y(j,i)-q(k,2));
            end
        end
    end
end

for i = 1:length(ch(:))
    if ch(i)>10
        ch(i) = 10;
    end
    if ck(i)>10
        ck(i) = 10;
    end
end

figure(1);
subplot(1,2,1);
scatter(q(:,1), q(:,2),'k');
hold on;
scatter(X(:), Y(:), 1, ch(:))
theta = linspace(pi,3*pi/2,100);
Xr = (L(1)+L(2))*sin(theta);
Yr = (L(1)+L(2))*cos(theta);
plot(Xr, Yr, 'r');

subplot(1,2,2);
scatter(q(:,1), q(:,2),'k');
hold on;
scatter(X(:), Y(:), 1, ck(:))
plot((L(1)+L(2))*sin(theta), (L(1)+L(2))*cos(theta), 'r');