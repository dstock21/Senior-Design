function [ir, e] = err(q, qr)
%% function that iterates through all the points in the reference gait set and picks the closest matching gait
%% Inputs q: joint positions, qr: reference gait inputs
err = inf;

% check for well formatted values
if isnan(q(1))
    ir = NaN;
    e = NaN;
    return
end

% Iterate through a set of values and find closes matching index
for k = 1:length(qr)
    temp = (q(1)-qr(k,1))^2 + (q(2) - qr(k,2))^2;
    if temp < err
        err = temp;
        ir = k;
    end
end

err = abs(q-qr(ir,:))-3;

for i = 1:length(err)
    if err(i) < 0
        err(i) = 0;
    end
end
e = 0.1*err.^2;