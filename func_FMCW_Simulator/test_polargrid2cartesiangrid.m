clc;
clear;
close ALL;
format compact;

%% test script

% define static targets in scene
[ target_attribute ] = func_target_attributes;

% define antenna gain pattern
[ antenna_gain_pattern ] = func_antenna_gain_pattern;
agp = antenna_gain_pattern;

% define chirp parameters
[ chirp_params ] = func_defineChirpParameters( 'best_range_res' );

[ radar_target_list,range_angle_heatmap_zeroVd,range_angle_range,range_angle_deg ] = ...
   func_fmcw_radar_simulator( target_attribute,agp,chirp_params );

% convert angular data to radians
range_angle_rad = deg2rad(range_angle_deg);

res_range = range_angle_range(2) - range_angle_range(1)
res_deg   = sind(range_angle_deg(2)) - sind(range_angle_deg(1))

k = 1;
for i=1:length(range_angle_range)   
    for j=1:length(range_angle_deg)
        
        r = range_angle_range(i);
        phi = range_angle_rad(j) + pi/2;
        int = range_angle_heatmap_zeroVd(i,j);
        
        data(k,:) = [r phi int];
        k = k + 1;     
    end 
end

% scale intesity data between [0 1]
data(:,4) = mat2gray(data(:,3));

figure
polarscatter(data(:,2),data(:,1),1,'filled');
pax = gca;
pax.ThetaLim = [0 180];

% create (x,y) grid cell array, aka 'heatmap'
heatmap = zeros(127,250);

k = 1;
for i=1:length(range_angle_range)
    
    for j=1:length(range_angle_rad)
        
        phi = range_angle_rad(j) + pi/2;
        x = range_angle_range(i)*cos(phi);
        y = range_angle_range(i)*sin(phi);
        int = 10*log10(range_angle_heatmap_zeroVd(i,j));
        
        ii = floor(x/res_range) + 125;
        jj = floor(y/res_range) + 1;
        
        val(k,:) = [ii jj x y int];
        k = k + 1;
        
        % define cartesian heatmap
        % Q: do I want this value in decibels ar linear scale? How will
        % this affect feature detection algorithm?
        heatmap(jj,ii) = int;  
    end  
end

% scale intesity data between [0 1]
val(:,6) = mat2gray(val(:,5));

figure
scatter(val(:,3),val(:,4),5,val(:,5),'fill')
colormap('jet')
colorbar
caxis([min(val(:,5)) max(val(:,5))])

figure
pcolor(heatmap)
shading flat;
colormap('jet')
colorbar
caxis([min(val(:,5)) max(val(:,5))])

fprintf('\tmax\t\tmin\n')
fprintf('ii\t%d\t\t%d\n',max(val(:,1)),min(val(:,1)));
fprintf('jj\t%d\t\t%d\n',max(val(:,2)),min(val(:,2)));

