function [mu, sigma2] = weightedSum(mu_x, sigma2_x, mu_y, sigma2_y, w)
%weightedSum performs a weighted sum of two Gaussian distributions
%
%Input
%   MU_X            The mean of the Gaussian random variable 1.
%   SIGMA2_X        The variance of the Gaussian random variable 1.
%   MU_Y            The mean of the Gaussian random variable 2.
%   SIGMA2_Y        The variance of the Gaussian random variable 2.
%   W               The weight of the Gaussian random variable 1.
%
%Output
%   MU              The mean of the Gaussian weighted sum.
%   SIGMA2          The variance of the Gaussian weighted sum.

mu = w*mu_x + (1 - w)*mu_y;
sigma2 = w*sigma2_x + (1 - w)*sigma2_y + w*(1 - w)*(mu_x - mu_y)^2;

end