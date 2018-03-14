close all;
clear all;

%% load data
R1 = 'I3:K2353';
R2 = 'M3:O2905';

X = xlsread('data.xlsx', R2);
t = X(:,2);
V = X(:,3);

%% filter
B = 0.9;
Vavg = zeros(size(V));
Vavg(1) = V(1);

for i = 2:length(V)
    Vavg(i) = B*Vavg(i-1) + (1-B)*V(i);
end

%% torque
offset = 1.85;
sens = (8*6/5.6);
torque = @(V) (V-offset)*sens;

Q = torque(V);
Qavg = torque(Vavg);

%% plot
figure(1);
hold on;
plot(1:length(V), Q);
plot(1:length(V), Qavg);
legend('unfiltered', 'filtered');

%% statistics
r1 = 10:150;
r2 = 650:1000;
r3 = 1700:1900;
r4 = 2600:length(V);

stat = @(d) [std(d(r1)), mean(d(r1));
             std(d(r2)), mean(d(r2));
             std(d(r3)), mean(d(r3));
             std(d(r4)), mean(d(r4))];

stat1 = stat(Q);
stat2 = stat(Qavg);