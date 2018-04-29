close all;
clear all;

%%
load('TSR_lin_BEST.mat');
des = 2.5;

t = (t-t(1))/1000000;

tr = find(abs(Q) > 0.9*des, 1);

% t0 = t(tr);
t0 = 0;
tf = 3;
r = find(t > t0, 1, 'first'):find(t<tf, 1, 'last');

lin = linspace(0,10, length(t))';
mu = mean(abs(Q(r))-lin(r));
sigma = std(abs(Q(r))-lin(r));
% mu = abs(mean(Q(r)));
% sigma = std(Q(r));


%%

figure(1);
plot((t-t(1)), abs(Q), 'k', 'LineWidth', 2);
hold on;
% plot((t-t(1)), des*ones(size(t)), '--r', 'LineWidth', 2);
% plot(t(r), (mu-sigma)*ones(size(r)), '--b', 'LineWidth', 2);
% plot(t(r), (mu+sigma)*ones(size(r)), '--b', 'LineWidth', 2);
plot((t-t(1)), lin, '--r', 'LineWidth', 2);
plot(t(r), lin(r)+mu+sigma, '--b', 'LineWidth', 2);
plot(t(r), lin(r)+mu-sigma, '--b', 'LineWidth', 2);
title('Torque Step Response', 'fontsize', 16);
xlabel('Time (s)', 'fontsize', 14);
ylabel('Torque (Nm)', 'fontsize', 14);
legend('Actual Torque', 'Desired Torque', 'Torque Error', 'Location', 'SouthEast');
xlim([0, 3]);
ylim([0,8.5]);
