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

%% plot
f = figure(1);
% set(f, 'position', [300, 200, 1000, 500]);
SH1 = subplot(1,2,1);
xAll = [xh-xh, xk-xh, xa-xh, xt-xh];
hold on;
Hlinks = plot(SH1, xAll(1, [1 3 5 7]), xAll(1,[2 4 6 8]), '-k');
Hlinks.XDataSource = 'Hlinksx';
Hlinks.YDataSource = 'Hlinksy';
Hlimbs = scatter(SH1, xAll(1, [1 3 5 7]), xAll(1,[2 4 6 8]), 5, 'r');
Hlimbs.XDataSource = 'Hlimbsx';
Hlimbs.YDataSource = 'Hlimbsy';
HTstr1 = ['Q = ' num2str(Mh(1)) ' N*m'];
HTstr2 = ['Q = ' num2str(Mk(1)) ' N*m'];
HTstr3 = ['Q = ' num2str(Ma(1)) ' N*m'];
Htext1 = text(SH1, xAll(1, 1) + 0.03, xAll(1, 2), HTstr1);
Htext2 = text(SH1, xAll(1, 3) + 0.03, xAll(1, 4), HTstr2);
%Htext3 = text(SH1, xAll(1, 5), xAll(1, 6), HTstr3);
xlim(SH1, [-0.45, 0.55]);
ylim(SH1, [-.9, 0.2]);
axis equal
xlabel('Position (m)');
ylabel('Height (m)');
title('Gait Simulation');

% SH2 = subplot(1,2,2);
% ylabel(SH1, 'Vertical Position (m)');
% xlabel(SH1, 'Horizontal Position (m)');
% title(SH1,'Patient Applied Torque')
% hold on;
% yyaxis left;
% Hqdoth = scatter(SH2, T(1), qdot(1,1), '.b');
% Hqdoth.XDataSource = 'HT';
% Hqdoth.YDataSource = 'Hqdothy';
% Hqdotk = scatter(SH2, T(1), qdot(1,2), '.g');
% Hqdotk.XDataSource = 'HT';
% Hqdotk.YDataSource = 'Hqdotky';
% ylabel(SH2, 'Angular Velocity (rad/s)');
% yyaxis right;
% 
% Hqh = scatter(SH2, T(1), q(1,1), '.r');
% Hqh.XDataSource = 'HT';
% Hqh.YDataSource = 'Hqhy';
% Hqk = scatter(SH2, T(1), q(1,2), '.k');
% Hqk.XDataSource = 'HT';
% Hqk.YDataSource = 'Hqky';
% ylabel(SH2, 'Joint Angle (rad)');
% xlim(SH2, [0,1]);
% legend(SH2, 'hip angular velocity', 'knee angular velocity', 'hip angle', 'knee angle');
% xlabel(SH2, 'Time (s)');
% title(SH2,'Angular Velocities');

filename = 'gait_simulation';
% Capture the plot as an image 
frame = getframe(f); 
im = frame2im(frame); 
[imind,cm] = rgb2ind(im,256); 
% Write to the GIF File 
imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
 
for i = 1:length(T)
    if i > 1
        pause(T(i)-T(i-1));
    end
    Hlinksx = xAll(i, [1 3 5 7]);
    Hlinksy = xAll(i, [2 4 6 8]);
    Hlimbsx = xAll(i, [1 3 5 7]);
    Hlimbsy = xAll(i, [2 4 6 8]);
    xlim(SH1, [-0.45, 0.55]);
    
    HTstr1 = ['Q = ' num2str(Mh(i)) ' N*m'];
    HTstr2 = ['Q = ' num2str(Mk(i)) ' N*m'];
    HTstr3 = ['Q = ' num2str(Ma(i)) ' N*m'];
    set(Htext1, 'string', HTstr1, 'position', xAll(i, 1:2) + [0.03,0]);
    set(Htext2, 'string', HTstr2, 'position', xAll(i, 3:4)+ [0.03,0]);
    %set(Htext3, 'string', HTstr3, 'position', xAll(i, 5:6));
    
%     HT = T(1:i);
%     Hqdothy = qdot(1:i,1);
%     Hqdotky = qdot(1:i,2);
%     Hqhy = q(1:i,1);
%     Hqky = q(1:i,2);
    refreshdata
    
    frame = getframe(f); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    imwrite(imind,cm,filename,'gif','WriteMode','append');
%     if i == length(T)-1
%         i = 1;
%     end
end
