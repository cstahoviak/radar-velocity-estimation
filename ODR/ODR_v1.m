function [ model, beta, cov_beta, iter ] = ODR_v1( data, d, beta0, ...
    delta0, weights, converge_thres, max_iter )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%%% TODO:
% 1. fine-tune Lagrange multiplier alpha
% 2. How to define scaling matrices S and T - ODR Guidebook??

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
        [ G, V, D ] = odr_getJacobian( radar_azimuth, delta, ...
            beta(:,k), weights, d );
    elseif p == 3
        [ G, V, D ] = odr_getJacobian3D( [radar_azimuth; radar_elevation], ...
            delta, beta(:,k), weights, d );
    else
        error('initial guess must be a 2D or 3D vector')
    end
    
%     disp('G ='); disp(G)
%     disp('V ='); disp(V)
%     disp('D ='); disp(D)
    
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
cov_beta = odr_getCovariance( G, V, D, eps, delta, weights, d );
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

