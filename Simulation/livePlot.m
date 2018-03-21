close all;
clear all;

%% connect serial
arduino = serial('COM4', 'BaudRate', 9600);
fopen(arduino);

cs = -10000; % start character

global L
L = [1;1;1];

% stored live variables
N = 20;
ka = zeros(N, 1);
kt = zeros(N, 1);
ha = zeros(N, 1);
ht = zeros(N, 1);
t = zeros(N, 1);

f1 = figure(1);
SH1 = subplot(2,4,[1:2, 5:5]);
title('Gait Characterization');
xlabel('Ankle X Position (m)');
ylabel('Ankle Y Position (m)');

SH2 = subplot(2,4,3);
Hka = plot(SH2, t, ka, '-k');
Hka.XDataSource = 't';
Hka.YDataSource = 'ka';
title('Knee Angle');
xlabel('Time (s)');
ylabel('Knee Angle (rad)');

SH3 = subplot(2,4,4);
Hkt = plot(SH3, t, kt, '-k');
Hkt.XDataSource = 't';
Hkt.YDataSource = 'kt';
title('Knee Torque');
xlabel('Time (s)');
ylabel('Knee Torque (Nm)');

SH4 = subplot(2,4,7);
Hha = plot(SH4, t, ha, '-k');
Hha.XDataSource = 't';
Hha.YDataSource = 'ha';
title('Hip Angle');
xlabel('Time (s)');
ylabel('Hip Angle (rad)');

SH5 = subplot(2,4,8);
Hht = plot(SH5, t, ht, '-k');
Hht.XDataSource = 't';
Hht.YDataSource = 'ht';
title('Hip Torque');
xlabel('Time (s)');
ylabel('Hip Torque (Nm)');

tic;
while true
    % order: cs; knee angle; knee torque; hip angle; hip torque;
    a = fscanf(arduino, '%f');
    
    if a == cs
        ka = [fscanf(arduino, '%f'); ka(1:end-1)];
        kt = [fscanf(arduino, '%f'); kt(1:end-1)];
        ha = [fscanf(arduino, '%f'); ha(1:end-1)];
        ht = [fscanf(arduino, '%f'); ht(1:end-1)];
        t = [toc; t(1:end-1)];
        
        ax = fk([ha; ka; 0]);
        subplot(2,4,[1:2,5:6]);
        hold on;
        axis equal;
        scatter(ax(1,2), ax(2,2), '.k');
    end
end