%% forward velocity kinematics of exoskeleton
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% q = 3x1 joint angle input
% qdot = 2x1 joint angle derivative (hip, knee respectively)
% L = 3xl link lengths (hip-knee, knee-ankle, ankle-foot)
% outputs - 
% v = 2x1 ankle velocity

function v = fvk(q, qdot)

J = computeJacobian(q);

v = J*qdot;