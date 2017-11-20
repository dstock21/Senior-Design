close all;
clearvars;

%% load gait data

% time, hip, knee, ankle, and toe position data respectively (cm to m)
T = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'B4:B109');
xh = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'E4:F109')/100;
xk = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'G4:H109')/100;
xa = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'K4:L109')/100;
xt = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'K4:L109')/100;

% velocities
vxh = xlsread('Winter_Appendix_data.xlsx','A2.Filtered_Marker_Kinematics', 'L5:L110');
vyh = xlsread('Winter_Appendix_data.xlsx','A2.Filtered_Marker_Kinematics', 'O5:O110');
vxa = xlsread('Winter_Appendix_data.xlsx','A2.Filtered_Marker_Kinematics', 'AG5:AG110');
vya = xlsread('Winter_Appendix_data.xlsx','A2.Filtered_Marker_Kinematics', 'AJ5:AJ110');

% joint angles (theta, omega, alpha: ankle, knee, hip)
dataq = xlsread('Winter_Appendix_data.xlsx','A4.Filtered_Marker_Kinematics', 'D5:L110');
q = dataq(:,[7 4 1])*pi/180; % degrees to radians
qdot = dataq(:,[8 5 2]);
qdotdot = dataq(:,[9 6 3]);

L = [norm(xh(1,:)-xk(1,:));
    norm(xk(1,:)-xa(1,:));
    norm(xa(1,:)-xt(1,:))];

x = xa-xh;


scatter(x(:,1), y(:,2), '.');