%% Friction compensation of exoskeleton
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% Fload = 2x1 Load force at each joint bearing
% global variables - 
% mu_bearing = 1x1 coefficient of friction for the bearing
% outputs - 
% Q = compensation torque

function Q = FRcomp(Fload, qdot, rBearing)

global mu_bearing qdotmin

qdot_hat = sign(qdot);
if abs(qdot(1)) < qdotmin
    qdot_hat(1) = 0;
end
if abs(qdot(2)) < qdotmin
    qdot_hat(2) = 0;
end
Q = -qdot_hat.*rBearing.*Fload*mu_bearing;
