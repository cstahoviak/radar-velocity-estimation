clear;
clc;
close ALL;

tic
for i=1:10
    testTimer()
    pause(1)
end
    

% for(i=1:10)
%     A(i).time = i;
%     A(i).x = i*10;
%     A(i).y = i*2;
%     A(i).point = [A(i).x A(i).y];
%     
% end