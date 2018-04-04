%% Gravity compensation of exoskeleton
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% q = 3x1 joint angle 
% m = 2x1 mass of hip-knee link and knee-ankle link
% global variables - 
% L = 2x1 link length
% g = 1x1 acceleration due to gravity
% outputs - 
% Q = compensation torque

function Q = Gcomp(q, m)

global L g

% equation uses extension as positive for the knee joint
q(2) = -q(2);

Q = [(m(1)*g*L(1)*sin(q(1)) + m(2)*g*L(2)*sin(q(1)+q(2)))/2;
    m(2)*g*L(2)*sin(q(1)+q(2))/2];
