function [mu, Sigma] = jointGaussian(mu_x, sigma2_x, sigma2_r)
%jointGaussian calculates the joint Gaussian density as defined
%in problem 1.3a. 
%
%Input
%   MU_X        Expected value of x
%   SIGMA2_X    Covariance of x
%   SIGMA2_R    Covariance of the noise r
%
%Output
%   MU          Mean of joint density 
%   SIGMA       Covariance of joint density


A = [1, 0; 1, 1];           % Calculated Linear transform matrix
b = [0; 0];                 % Calculated Constant part of the affine transformation

mu_overall = [mu_x; 0];     % Rearranging different mean values for affine transform
Sigma_overall = [sigma2_x, 0; 0, sigma2_r];     % Rearranging different variances for affine transform

[mu, Sigma] = affineGaussianTransform(mu_overall, Sigma_overall, A, b);

end