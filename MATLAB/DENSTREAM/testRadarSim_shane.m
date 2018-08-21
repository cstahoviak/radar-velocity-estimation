%% Test Radar Simulator

clc;
clear;
close ALL;

Ntargets_RDRSIM = 5;
    
RCS = 50*ones(Ntargets_RDRSIM,1);       % radar cross section        [?]
points = [0 10; 0 4; 0 3; 0 2; 5 5];    % 2D target locations        [m]
velocity = zeros(Ntargets_RDRSIM,1);    % target velocity            [m/s]
AoA = zeros(Ntargets_RDRSIM,1);         % target direction of travel [deg]
                                        %   0 deg   - moving left
                                        %   90 deg  - moving away from sensor 
                                        %   180 deg - moving right 
                                        %   270 deg - moving towards sensor

target_attribute = [RCS, points, velocity, AoA];
[ targets ] = func_fmcw_radar_simulator_2018_0721( target_attribute );

fprintf('MAX SNR = %f\n',max(targets(:,1)));
fprintf('MIN SNR = %f\n',min(targets(:,1)));

i = numel(targets) / 5;
while i ~= 0
    if(targets(i,1) < 0)
        targets(i,:) = [];
    end
    i = i-1;
end


figure(1)
scatter(points(:,1),points(:,2),20,'kx'); hold on;

i = 1;
while i <= numel(targets) / 5;
    dot_size = targets(i,1) *2;
    scatter(targets(i,2),targets(i,3),dot_size,'filled')
    i = i + 1
end
%scatter(targets(:,2),targets(:,3),10,'filled')