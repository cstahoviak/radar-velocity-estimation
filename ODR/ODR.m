function [ model, beta ] = ODR( radar_angle, radar_doppler, ...
    d, beta0, delta0, weights )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%%% TODO:
% 1. fine-tune Lagrange multiplier alpha
% 2. How to define scaling matrices S and T - ODR Guidebook??

Niter = 150;
Ntargets = size(delta0,1);
p = size(beta0,1);

% [ S, T ] = ODR_getScalingMatrices();
% weights = ODR-getWeights( sigma_vr, sigma_theta, radar_intensity );

alpha = 0.1;  % Lagrange multiplier

S = eye(p);             % s scaling matrix
T = eye(Ntargets);      % t scaling matrix

% initialize
beta = zeros(2,Niter);
delta = zeros(Ntargets,Niter);
s = zeros(2,Niter);

beta(:,1) = beta0;
delta(:,1) = delta0;
s(:,1) = ones(2,1);

% set fminunc options
options = optimoptions('fminunc','Display','none',...
    'Algorithm','quasi-newton');

for i=1:Niter
    
    disp([i, s(:,i)'])
    
    % get Jacobian matrices
    [ G, V, D ] = ODR_getJacobian( radar_angle, delta(:,i), ...
        beta(:,i), weights, d );
    
    % defined to simplify the notation in objectiveFunc
    P = V'*V + D^2 + alpha*T^2;
    
    doppler_predicted =  simulateRadarDoppler2D(beta(:,i), ...
    radar_angle', zeros(Ntargets,1), delta(:,i));

    % re-calculate epsilon
    eps = doppler_predicted - radar_doppler';
    
    % anonymous function defined within interation loop in order to use 
    % current values of G, V and D
    f = @(s) objectiveFunc(s,G,V,D,P,eps,delta(:,i));
    
    s(:,i+1) = fminunc(f,s(:,i),options);
    t = -inv(P)*(V'*eps + D*delta(:,i) + V'*G*s(:,i+1));
    
    % use s and t to iteratively update beta and delta, respectively
    beta(:,i+1) = beta(:,i) + s(:,i+1);
    delta(:,i+1) = delta(:,i) + t;
    
end

% k = 1;
% converge_thres = 0.001;
% while (abs(s(1,k)) > converge_thres) ||  (abs(s(2,k)) > converge_thres)
%     
%     % get Jacobian matrices
%     [ G, V, D ] = ODR_getJacobian( radar_angle, delta, ...
%         beta(:,k), weights, d );
%     
%     % defined to simplify the notation in objectiveFunc
%     P = V'*V + D^2 + alpha*T^2;
%     
%     doppler_predicted =  simulateRadarDoppler2D(beta(:,k), ...
%     radar_angle', zeros(Ntargets,1), delta);
% 
%     % re-calculate epsilon
%     eps = doppler_predicted - radar_doppler';
%     
%     % anonymous function defined within interation loop in order to use 
%     % current values of G, V and D
%     f = @(s) objectiveFunc(s,G,V,D,P,eps,delta);
%     
%     s(:,k+1) = fminunc(f,s(:,k),options);
%     t = -inv(P)*(V'*eps + D*delta + V'*G*s(:,k+1));
%     
%     % use s and t to iteratively update beta and delta, respectively
%     beta(:,k+1) = beta(:,k) + s(:,k+1);
%     delta = delta + t;
%     
%     disp([k, s(:,k+1)'])
%     
%     k = k + 1;
% end

model = beta(:,end);

end

function [ fval ] = objectiveFunc(s, G, V, D, P, eps, delta)

if (size(G,1) ~= size(V,1)) && (size(G,1) ~= size(V,1)) ...
        && (size(G,1) ~= size(P,1)) && (size(G,2) ~= size(s,1))
    
    error('objectiveFunc: matrix size mismatch')
    
else
    n = size(G,1);

%     f = sqrt(eye(n) - V*inv(P)*V')*G*s - 1./sqrt(eye(n) - V*inv(P)*V') * ...
%         (eps + V*inv(P)*(V'*eps + D*delta));
    
    f = (eye(n) - V*inv(P)*V')^(1/2)*G*s - (eye(n) - V*inv(P)*V')^(-1/2) * ...
        (-eps + V*inv(P)*(V'*eps + D*delta));
    
    fval = norm(f);
end

end

