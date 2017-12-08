%% Resistive torque
% apply a resistive torque to the hip and knee joints based on a resistive
% force at the ankle that is proportional to the distance between the
% current gait position and the gait cycle
%
% inputs - 
% X = kx2 current position
% outputs - 
% state = 0: not in a state yet, 1: leg moving forward, 2: leg moving
% backward

function state = changeState(X, state0)

global nHyst

if nHyst <= size(X,1)
    v = diff(X(1:nHyst,:));
    dir = sign(v(:,1));

    if all(dir > 0)
        state = 1;
    elseif all(dir < 0)
        state = 2;
    else
        state = state0;
    end
else
    state = state0;
end