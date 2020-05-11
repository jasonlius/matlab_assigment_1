function [mu_y, Sigma_y] = approximateMeanAndCovariance(mu_x, Sigma_x, h, R, type)
%approximateMeanAndCovariance takes a Gaussian density and a transformation 
%function and calculates the mean and covariance of the transformed density.
%
%Inputs
%   MU_X        [m x 1] Expected value of x.
%   SIGMA_X     [m x m] Covariance of x.
%   H           [Function handle] Function which maps a [m x 1] dimensional
%               vector into another vector of size [n x 1].
%   R           [n x n] Covariance of y.
%   TYPE        [1 x 1] Type of transformation required
%
%Output
%   MU_Y        [n x 1] Mean of y.
%   SIGMA_Y     [n x n] Covariance of y.

switch type
	case 'EKF'
        [hx, Hx] = h(mu_x);
        mu_y = hx;
        Sigma_y = Hx*Sigma_x*Hx' + R;
        
    case 'UKF'
        [SP,W] = sigmaPoints(mu_x,Sigma_x,type);
        mu_y = zeros(size(mu_x));
        for i = 1:size(W,2)
            mu_y = mu_y + h(SP(:,i))*W(i);
        end
        Sigma_y = R;
        for i = 1:size(W,2)
            Sigma_y = Sigma_y + (h(SP(:,i)) - mu_y)*(h(SP(:,i)) - mu_y).'*W(i);
        end
        
    case 'CKF'
        [SP,W] = sigmaPoints(mu_x,Sigma_x,type);
        mu_y = zeros(size(mu_x));
        for i = 1:size(W,2)
            mu_y = mu_y + h(SP(:,i))*W(i);
        end
        Sigma_y = R;
        for i = 1:size(W,2)
            Sigma_y = Sigma_y + (h(SP(:,i)) - mu_y)*(h(SP(:,i)) - mu_y).'*W(i);
        end
    otherwise
        error('Incorrect type of non-linear Kalman filter')
end
end