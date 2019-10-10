function [ model, beta, cov_beta, iter ] = ODR_v5( data, d, beta0, ...
    sigma, weights, s, converge_thres, max_iter, get_covar )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% d - error variance ratio vector - 
%   [sigma_eps/sigma_dtheta; sigma_eps/sigma_dphi] - (m x 1)

%%% NOTE:
% 1. ODR is non-deterministic in its current form - it will not converge to
%    *exactly* the same solution given for a given set of inputs.

%%% TODO:
% 1. fine-tune Lagrange multiplier alpha
% 2. How to define scaling matrices S and T - ODR Guidebook??

% 3. Only calculate D once per estimate, NOT once per iteration - (DONE)
% 4. Update how the delta vector is defined: (DONE)
%       old: delta = [dtheta_1, dtheta_2, ..., dtheta_n, dphi_1, dphi_2,..., dphi_n]
%       new: delta = [dtheta_1, dphi_1, dtheta_2, dphi_2, ..., dtheta_n, dphi_n]
% 5. Verify weighting of Jacobian block-elements - Am I multiplying the
%    weights in the correct way? - YES
% 6. Investigate optimal value of scale factor for matrix S (currently 10)
% 7. Update convergence criteria - 2-norm of doppler residual instead of
%    norm of step size - see ODR-1989.
% 8. Implement final step of ODR-1987 to update alpha value at each iteration

% unpack data (into column vectors)
radar_doppler   = data(:,1);
radar_azimuth   = data(:,2);
radar_elevation = data(:,3);

Ntargets = size(radar_doppler,1);
p = size(beta0,1);
m = size(d,1);

% [ S, T ] = ODR_getScalingMatrices();
S = diag(s);           % s scaling matrix - 10 empirically chosen
T = eye(Ntargets*m);    % t scaling matrix
alpha = 1;          % Lagrange multiplier

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

% construct E matrix - E = D^2 + alpha*T^2 (ODR-1987 Prop. 2.1)
E = D^2 + alpha*T^2;
Einv = inv(E);

% initialize
beta      = NaN*ones(p,max_iter); 
beta(:,1) = beta0;
delta     = delta0;

k = 1;
while norm(s) > converge_thres
    
    % get Jacobian matrices
    if p == 2
        [ G, V, M ] = odr_getJacobian2D_v2( radar_azimuth, delta, ...
            beta(:,k), weights, E );
        
        doppler_predicted = simulateRadarDoppler2D(beta(:,k), ...
            radar_azimuth, zeros(Ntargets,1), delta);
        
    elseif p == 3
        [ G, V, M ] = odr_getJacobian3D_v2( [radar_azimuth; radar_elevation], ...
            delta, beta(:,k), weights, E, k );
        
        doppler_predicted = simulateRadarDoppler3D(beta(:,k), ...
            radar_azimuth, radar_elevation, zeros(Ntargets,1), delta);
    else
        error('initial guess must be a 2D or 3D vector')
    end
    
    if k == 1
%         disp('G_v5 ='); disp(G(1:p,1:p))     % G being computed correctly
%         disp('V_v5 ='); disp(V(1:2*m,1:2*m))
    end
    
    % re-calculate epsilon
    eps = doppler_predicted - radar_doppler;
    
    % update the (nX1) and (nmx1) elements of G
    g1 = weights.*eps;
    g2 = D*delta;
    
    % form the elements of the linear least squares problem
    Gbar = M*G;
    y = -M*(eps - V*Einv*D*delta);
%     y = -M*(g1 - V*Einv*D*g2);
    
    % Compute s via QR factorization of Gbar
    [Q,R] = qr(Gbar,0);
    s = R\(Q'*y);
    t = -Einv*(V'*M^2*(eps + G*s - V*Einv*D*delta) + D*delta);
%     t = -Einv*(V'*M^2*(g1 + G*s - V*Einv*D*g2) + D*g2);
    
    % use s and t to iteratively update beta and delta, respectively
    beta(:,k+1) = beta(:,k) + S*s;
    delta = delta + T*t;
    
%     disp([k, s', norm(s)])
    k = k + 1;
    
    if k > max_iter
%         disp([k, s'])
        disp('ODR_v5: max iterations reached')
        break
    end
end

% remove NaN columns from beta matrix
idx = ~isnan(beta(1,:));
beta = beta(:,idx);

model = beta(:,k);
if get_covar
    cov_beta = odr_getCovariance( Gbar, D, eps, delta, weights );
else
    cov_beta = NaN*ones(p);
end
iter = k-1;

end

