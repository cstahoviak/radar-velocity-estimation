clc;
clear;
close ALL;

%% TEST ONE

x = -5:5;
y = -5:5;
[xx,yy] = meshgrid(x,y);

circ = xx.^2 + yy.^2;

u = zeros(size(xx));
u( circ <= (5+0.5)^2 & circ > (4+0.5)^2 ) = 1
idx = u > 0;
idx2 = find(u > 0);

v = u(:);
v = v(idx2);

figure(1)
imagesc(u)

%% TEST 2

x = -110:110;
y = -110:110;
[xx,yy] = meshgrid(x,y);
u = zeros(size(xx));
u((xx.^2+yy.^2)<100^2)=1;   % radius 100, center at the origin
% hard boundary
figure(2)
imagesc(u)
% weight the points: point itself; average of nearest neighbors;
% averaged of diagonal neighbors.  These must add up to 1.
wp = .4;  wn = .4;  wd = .2;
ind = 2:length(x)-1;
u(ind,ind) = wp*u(ind,ind) ...
  + (wn/4)*(u(ind-1,ind  ) + u(ind+1,ind  ) + u(ind  ,ind-1) + u(ind  ,ind+1) ) ...
  + (wd/4)*(u(ind-1,ind-1) + u(ind-1,ind+1) + u(ind+1,ind-1) + u(ind+1,ind+1) );
% extended boundary
figure(3)
imagesc(u)
