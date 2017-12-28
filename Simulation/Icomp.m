%% inertia compensation of exoskeleton
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% q = 3x1 joint angle 
% qdot = 2x1 joint angle derivative
% qdotdot = 2x1 joint angle second derivative
% m = 2x1 mass of hip-knee link and knee-ankle link
% global variables - 
% L = 2x1 link length 
% outputs - 
% Q = compensation torque

function Q = Icomp(q, qdot, qdotdot, m)

global L

% equation uses extension as positive for the knee joint
q(2) = -q(2);

M = zeros(2,2);

M(1,1) = m(1)*L(1)^2/2 + m(2)*(L(1)^2 + cos(q(2))*L(1)*L(2)+L(2)^2/4);
M(1,2) = m(2)*(L(2)^2/4+L(1)*L(2)*cos(q(2))/2);
M(2,1) = m(2)*(L(2)^2/4+L(1)*L(2)*cosx(q(2))/2);
M(2,2) = m(2)*L(2)^2/4;

V1 = [-m(2)*sin(q(2))*L(1)*L(2); 
    0];

V2 = [0, -m(2)*L(1)*L(2)*sin(q(2))/2;
    m(2)*L(1)*L(2)*sin(q(2))/2, 0];

Q = M*qdotdot + V1*qdot(1)*qdot(2) + V2*(qdot.^2);
