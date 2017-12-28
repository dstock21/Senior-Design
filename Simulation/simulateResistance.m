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

Q = zeros(size(x));
Xcomp = zeros(size(x));
states = zeros(size(x,1),1);
states(1) = state0;
for i = 2:size(Q,1)
    if i > nHyst
        states(i) = changeState(X(i:-1:(i-nHyst+1),:),states(i-1));
    else
        states(i) = changeState(X(i:-1:1), states(i-1));
    end
    [Q(i,:), Xcomp(i,:)] = resistance(q(i,:)',X(i,:),v(i,:),states(i));
end

%% plot
f = figure(1);
set(f, 'position', [300, 200, 1000, 500]);
SH1 = subplot(1,2,1);
hold on;
Hbase = scatter(SH1, x(:,1), x(:,2), 5, 'r');
Hcomp = scatter(SH1, Xcomp(1,1), Xcomp(1,2), 10, 'b');
Hcomp.XDataSource = 'Hcompx';
Hcomp.YDataSource = 'Hcompy';
Hcurr = scatter(SH1, X(1,1), X(1,2), 5, 'k');
Hcurr.XDataSource = 'Hcurrx';
Hcurr.YDataSource = 'Hcurry';
HTstr1 = ['state = ' num2str(states(1))];
HTstr2 = ['Qhip = ' num2str(Q(1,1)) ' N*m'];
HTstr3 = ['Qknee = ' num2str(Q(1,2)) ' N*m'];
min([x(:,1); X(:,1)])
max([x(:,2); X(:,2)])
Htext1 = text(SH1, min([x(:,1); X(:,1)]), max([x(:,2); X(:,2)])+0.25, HTstr1);
Htext2 = text(SH1, min([x(:,1); X(:,1)])+0.15, max([x(:,2); X(:,2)])+0.25, HTstr2);
Htext3 = text(SH1, min([x(:,1); X(:,1)])+0.4, max([x(:,2); X(:,2)])+0.25, HTstr3);
axis equal
xlabel(SH1,'Ankle Horizontal Position(m)');
ylabel(SH1,'Ankle Vertical Position(m)');
legend(SH1, 'Reference Gait', 'Corresponding Reference Gait','Measured Gait','Location','southeast');
title(SH1,'Gait Comparison');
SH2 = subplot(1,2,2);
hold on;
HQh = scatter(SH2, T(1), Q(1,1), '.r');
HQh.XDataSource = 'HT';
HQh.YDataSource = 'HQhy';
HQk = scatter(SH2, T(1), Q(1,2), '.k');
HQk.XDataSource = 'HT';
HQk.YDataSource = 'HQky';
ylabel(SH2, 'Torque (N*m)');
xlim(SH2, [0,T(end)]);
legend(SH2, 'Hip Joint', 'Knee Joint');

xlabel(SH2, 'Time (s)');
title(SH2,'RehabiliGait Resistive Torque');
for i = 1:length(T)
    if i > 1
        pause((T(i)-T(i-1))*5);
    end
    Hcompx = Xcomp(i,1);
    Hcompy = Xcomp(i,2);
    Hcurrx = X(1:i,1);
    Hcurry = X(1:i,2);
    
    HTstr1 = ['state = ' num2str(states(i))];
    HTstr2 = ['Qhip = ' num2str(Q(i,2)) ' N*m'];
    HTstr3 = ['Qknee = ' num2str(Q(i,2)) ' N*m'];
    set(Htext1, 'string', HTstr1);
    set(Htext2, 'string', HTstr2);
    set(Htext3, 'string', HTstr3);
    
    HT = T(1:i);
    HQhy = Q(1:i,1);
    HQky = Q(1:i,2);
    refreshdata
end

