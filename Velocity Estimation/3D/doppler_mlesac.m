function [ model, inlier_idx, scores ] = doppler_mlesac( data, n, p, t, ...
    converge_thres, max_iter, sigma_vr)

%%% Inputs:
% data - a set of observed data points
% model - a model that can be fitted to data points (not used)
% n - the minimum number of data values required to fit the model
% max_iter - the maximum number of iterations allowed in the algorithm
% t - a threshold value for determining when a data point fits a model
% d - the number of close data values required to assert that a model fits 
%     well to data (not used)
% p - probability of sampling at least one good inlier set

radar_doppler   = data(:,1);
radar_azimuth   = data(:,2);
radar_elevation = data(:,3);

Ntargets = size(data,1);

bestScore   = -1e6;   % large negative number
bestInliers = [];
bestModel   = [];
scores      = NaN*ones(max_iter,1);
dll_incr    = inf;

k = 1;      % valid sample iteration number
j = 1;      % score count

while abs(dll_incr) > converge_thres && k < max_iter
    
    % randomly sample n observations from data
    sample = datasample(data,n);
    is_valid = is_data_valid( sample );
    
    if is_valid
        doppler = sample(:,1);
        azimuth = sample(:,2);
        elevation = sample(:,3);
        
        % valid pair of targets sampled - fit the model
        model = doppler2BodyFrameVelocities3D( doppler, ...
            azimuth, elevation);
        
        % generate predicted doppler measurements given the model
        doppler_predicted = simulateRadarDoppler3D( model, radar_azimuth, ...
            radar_elevation, zeros(Ntargets,1), zeros(2*Ntargets,1));
        
        % evaluate the data log-likelihood given the model
        eps_sq = (doppler_predicted - radar_doppler).^2;
        score = -1/(2*sigma_vr^2)*sum(eps_sq);
        
        if score > bestScore
            % this model better explains the data
            dll_incr = score - bestScore;
            bestScore = score;
            bestInliers = (sqrt(eps_sq) < t);
            bestModel = model;

            scores(j) = score;
            j = j+1;

            % evaluate stopping criteria - not yet used
%             Ninliers = sum(bestInliers);
%             w = Ninliers/Ntargets;
%             k1 = log(1-0.95)*log(1-w^2);
%             k2 = log(1-0.75)*log(1-w^2);
%             k3 = log(1-0.50)*log(1-w^2);
        end
        k = k + 1;
    end
end

% remove NaN values from scores
idx_nan = isnan(scores);
scores = scores(~idx_nan);

% return best model and inliers
model = bestModel;
inlier_idx = bestInliers;

% % options = optimoptions('lsqnonlin','Display','none');
% f = @(X) ols_func(X, radar_doppler(bestInliers), radar_azimuth(bestInliers));
% 
% % solve OLS problem on inlier set
% model = lsqnonlin(f,bestModel);
% inlier_idx = bestInliers;

end

function [ is_valid ] = is_data_valid( data )

    radar_azimuth = data(:,2);
    radar_elevation = data(:,3);

    [ numAzimuthBins, ~ ] = getNumAngleBins( radar_azimuth );
    [ numElevBins, ~ ] = getNumAngleBins( radar_elevation );

    if numAzimuthBins + numElevBins > 4
        is_valid = true;
    else
        is_valid = false;
    end

end

