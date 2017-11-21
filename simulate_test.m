% simulate.m - 
% simulates exoskeleton kinematics
close all;
clear all;
home;

%% initialize
% 12-17", 14-19"
global L

L = [15*0.0254; 17*0.0254; 6*0.0254];

figure(1);
x = [0,0,0,L(3)];
y = [0,-L(1),-L(1)-L(2),-L(1)-L(2)];
hold on;
axis equal;
H1 = plot(x, y, '-k');
H2 = scatter(x, y, 0.05, 'b');
H3 = plot(x, y, '-r');
H4 = scatter(x, y, 0.05, 'b');


%% plot random configurations
q = zeros(3,1);
q(1) = -20+ 120*rand();
q(2) = 100*rand();
q(3) = q(2)-q(1);
q = q*pi/180; % to radians

X = fk(q);
q_test = ik(X(:,2));
Xtest = fk(q_test);

set(H1, 'XData', [0, X(1,1:3)]);
set(H1, 'YData', [0, X(2,1:3)]);
set(H2, 'XData', [0, X(1,1:3)]);
set(H2, 'YData', [0, X(2,1:3)]);   
set(H3, 'XData', [0, Xtest(1,1:3)]);
set(H3, 'YData', [0, Xtest(2,1:3)]);
set(H4, 'XData', [0, Xtest(1,1:3)]);
set(H4, 'YData', [0, Xtest(2,1:3)]);

if abs(q-q_test) > 0.01
    [q, q_test]
end