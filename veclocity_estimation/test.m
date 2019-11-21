clc;
clear;

N = 5;

for i=1:(N-2)
    for j=i+1:(N-1)
        for k=j+1:N
            disp([i, j, k])
        end
    end
end