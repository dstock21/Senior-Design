%% inverse kinematics of exoskeleton
% angle flexion is considered positive
% zero pose is thigh, shank perpendicular to ground, foot parallel to
% ground. 
%
% inputs - 
% X = 2x1 ankle position
% L = 3xl link lengths (hip-knee, knee-ankle, ankle-foot)
% outputs - 
% q = 3x1 joint angle output
% note: q3 output is joint angle that makes foot parallel to ground
% note: for low magnitude q2 (+- 10 degrees), both flexion and extension
% are valid answers. The flexion joint angle is always returned.

function [q, possible] = ik(X)

global L

%   % check validitiy
L1 = L(1);
L2 = L(2);
% hip to ankle distance
L3 = sqrt(X(1)^2+X(2)^2);

possible = L3 <= L1+L2;

q = zeros(3,1);

if (possible)
    %% calculate
    theta = atan2(X(1), -X(2));
    % acos ensures phi > 0, meaning knee always in flexion
    phi = acos((L1^2+L3^2-L2^2)/(2*L1*L3));

    q(1) = theta+phi;
    q(2) = pi - acos((L1^2+L2^2-L3^2)/(2*L1*L2));
    q(3) = q(2)-q(1);
end
