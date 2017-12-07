%% Torque output
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% q = 3x1 joint angle input
% F = 2x1 Force applied to ankle
% L = 3xl link lengths (hip-knee, knee-ankle, ankle-foot)
% outputs - 
% Q = 2x1 torque applied to hip and knee

function Q = torque(q, F)

J = computeJacobian(q);

Q = J'*F;