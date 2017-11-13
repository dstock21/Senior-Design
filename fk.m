%% forward kinematics of exoskeleton
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% q = 3x1 joint angle input
% L = 3xl link lengths (hip-knee, knee-ankle, ankle-foot)
% outputs - 
% X = 3x2 knee, ankle, and end of foot positions


function X = fk(q, L)

X = zeros(3,2);
X(1,:) = L(1)*[sin(q(1)), -cos(q(1))];
theta2 = q(1)-q(2);
X(2,:) = X(1,:) + L(2)*[sin(theta2), -cos(theta2)];
theta3 = theta2 + pi/2 + q(3);
X(3,:) = X(2,:) + L(3)*[sin(theta3), -cos(theta3)];