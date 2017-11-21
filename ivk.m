%% inverse velocity kinematics of exoskeleton
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% q = 3x1 joint angle input
% v = 2x1 ankle velocity
% L = 3xl link lengths (hip-knee, knee-ankle, ankle-foot)
% outputs - 
% qdot = 2x1 joint angle derivative (hip, knee respectively)

function qdot = ivk(q, v)

J = computeJacobian(q);
Jplus = pinv(J);

qdot = Jplus*v;