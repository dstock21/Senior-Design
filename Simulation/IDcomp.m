%% Disc inertia compensation of exoskeleton
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% qdotdot = 2x1 joint angle second derivative
% J = 2x1 total moment of inertia of joint gear box and disc brake
% outputs - 
% Q = compensation torque

function Q = IDcomp(qdotdot, J)

Q = -qdotdot.*J;
