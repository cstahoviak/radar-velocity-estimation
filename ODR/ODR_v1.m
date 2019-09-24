function [ model, beta, cov_beta, iter ] = ODR_v1( data, d, beta0, ...
    sigma, weights, converge_thres, max_iter, get_covar )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% d - error variance ratio vector - 
%   [sigma_eps/sigma_dtheta; sigma_eps/sigma_dphi] (m x 1)

%%% TODO:
% 1. fine-tune Lagrange multiplier alpha
% 2. How to define scaling matrices S and T - ODR Guidebook??

% 3. Only calculate D once per estimate, NOT once per iteration
% 4. Update how the delta vector is defined:
%       old: delta = [dtheta_1, dtheta_2, ..., dtheta_n, dphi_1, dphi_2,..., dphi_n]
%       new: delta = [dtheta_1, dphi_1, dtheta_2, dphi_2, ..., dtheta_n, dphi_n]
% 5. Verify weighting of Jacobian block-elements - Am I multiplying the
%    weights in the correct way?

% unpack data (into column vectors)
radar_doppler   = data(:,1);
radar_azimuth   = data(:,2);
radar_elevation = data(:,3);

Ntargets = size(radar_doppler,1);
p = size(beta0,1);
m = size(d,1);

% [ S, T ] = ODR_getScalingMatrices();
S = 5*eye(p);           % s scaling matrix - 10 empirically chosen
T = eye(Ntargets*m);    % t scaling matrix
alpha = 0.001;          % Lagrange multiplier

if p == 2
    % init delta vector
    delta0 = normrnd(0,sigma,[Ntargets,1]);
    
    % construct weighted diagonal matrix D
    D = diag(weights * d);
    
elseif p == 3
    % init delta vector - "interleaved" vector
    delta0 = [normrnd(0,sigma(1),[1,Ntargets]); normrnd(0,sigma(2),[1,Ntargets])];
    delta0 = delta0(:);

    % construct weighted block-diagonal matrix D
    D = diag(d);
    Drep = repmat(D, 1, Ntargets);
    Dcell = mat2cell(Drep, m, repmat(m,1,Ntargets));
    Dblk = blkdiag(Dcell{:});
    weights_diag = diag(repelem(weights,m));
    D = weights_diag*Dblk;
    
else
    error('initial guess must be a 2D or 3D vector')
end

% initialize
beta(:,1) = beta0;
delta     = delta0;
s         = ones(p,1);

% set fminunc options
options = optimoptions('fminunc','Display','none',...
    'Algorithm','quasi-newton');

k = 1;
while norm(s) > converge_thres
    
    % get Jacobian matrices
    if p == 2
        [ G, V ] = odr_getJacobian( radar_azimuth, delta, ...
            beta(:,k), weights );
    elseif p == 3
        [ G, V ] = odr_getJacobian3D( [radar_azimuth; radar_elevation], ...
            delta, beta(:,k), weights );
    else
        error('initial guess must be a 2D or 3D vector')
    end
    
%     disp('G ='); disp(G)
%     disp('V ='); disp(V)
    
    % defined to simplify the notation in objectiveFunc
    P = V'*V + D^2 + alpha*T^2;
    
    if p == 2
        doppler_predicted =  simulateRadarDoppler2D(beta(:,k), ...
            radar_azimuth, zeros(Ntargets,1), delta);
    elseif p == 3
        doppler_predicted =  simulateRadarDoppler3D(beta(:,k), ...
            radar_azimuth, radar_elevation, zeros(Ntargets,1), delta);
    else
        error('initial guess must be a 2D or 3D vector')
    end
    
    % re-calculate epsilon
    eps = doppler_predicted - radar_doppler;
    
    % anonymous function defined within interation loop in order to use 
    % current values of G, V, D, eps and delta
    f = @(s) objectiveFunc(s,G,V,D,P,eps,delta);
    
    s = fminunc(f,s,options);
    t = -inv(P)*(V'*eps + D*delta + V'*G*s);
    
    % use s and t to iteratively update beta and delta, respectively
    beta(:,k+1) = beta(:,k) + S*s;
    delta = delta + T*t;
    
%     disp([k, s', norm(s)])
    k = k + 1;
    
    if k > max_iter
%         disp([k, s'])
%         disp(beta)
        disp('ODR_3D: max iterations reached')
        break
    end
end 

model = beta(:,end);
if get_covar
    cov_beta = odr_getCovariance( G, V, D, eps, delta, weights, d );
else
    cov_beta = NaN*ones(p);
end
iter = k-1;

end

function [ fval ] = objectiveFunc(s, G, V, D, P, eps, delta)

if (size(G,1) ~= size(V,1)) && (size(G,1) ~= size(V,1)) ...
        && (size(G,1) ~= size(P,1)) && (size(G,2) ~= size(s,1))
    
    error('objectiveFunc: matrix size mismatch')
    
else
    n = size(G,1);
    
    f = (eye(n) - V*inv(P)*V')^(1/2)*G*s - (eye(n) - V*inv(P)*V')^(-1/2) * ...
        (-eps + V*inv(P)*(V'*eps + D*delta));
    
    fval = norm(f);
end

end

