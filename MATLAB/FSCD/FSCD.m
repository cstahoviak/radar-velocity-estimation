function [ L ] = FSCD( grid,thres )
% Features from Scattering Centers Detector (FSCD)
%   Detailed explanation goes here

% define circular pattern used to test candidate LM grid cells
pattern = [-1  3;  0  3;  1  3;  2  2;
            3  1;  3  0;  3 -1;  2 -2;
            1 -3;  0 -3; -1 -3; -2 -2;
           -3 -1; -3  0; -3  1; -2  2];

LM_index = 1;

for i=4:(M-4)
    
    for j=4:(N-4)
        
        % consider only high-intensity cells as landmark candidates
        if grid(i,j) > thres
            k = 1;
            flag = true;
            while flag && k <= size(pattern,1)
                % grid cell intensity value in pattern larger than center
                if grid(i+pattern(k,1),j+pattern(k,2)) > grid(i,j)
                    flag = false;
                end
                k = k+1;
            end
            % all intensity values in pattern lower than center
            % Q: Do I need both conditions?...
            if k > size(pattern,1) && flag == true
                % assign LM index location to current location of grid cell
                LM(LM_index,1:2) = [i,j];
                LM_index = LM_index+1;
                
                % assign LM descriptor using BASD method - Do this AFTER LM "clustering"
                
            end
                
        end % end if grid(i,j) > thres
        
    end  % end j

end  % end i

% "The detector observes landmarks which are close to each other due to
% pattern design. A simple clustering with linear complexity is used in
% post-processing to combine these adjacent landmarks into a single LM."

% assign BASD descriptor to each landmark in the "clustered" LM set
% [ ] = BASD( );


end

