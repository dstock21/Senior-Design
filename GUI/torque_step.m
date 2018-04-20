close all;
clear all;

%%
s = serial('COM1');
fopen(s);

fprintf('s'); % start sending data

while 1
    ba = s.BytesAvailable;
    if ba >= 4
        break
    end
end

out = fscanf(s, '%f');
while (out~= -10000)
    out = fscanf(s, '%f');
    continue
end

%% start retrieving data
t = fscanf(s, '%f');
Q = fscanf(s, '%f');

while 1
    ba = s.BytesAvailable;
    if ba >= 12
        break
    end
    
    out = fscanf(s, '%f');
    if out == -5000
        break;
    elseif out == -10000
        t = [t; fscanf(s, '%f')];
        Q = [Q; fscanf(s, '%f')];
    end
end

%% plot

figure(1);
plot(t, Q);

%% end
fclose(s)
delete(s)
clear s