%% SSY345: Sensor Fusion and Non-linear Filtering
% Home Assignment 1 (Implementation)
% 
% Author: Sourab Bapu Sridhar

clc
clear
close all
%% Problem 1: sigmaEllipse2D

%Here we give parameters for a Gaussian density. The parameter mu is the mean, and P is the covariance matrix.
mu = [-2; 1];
P = [4, -2; -2 2];

%Call your function.
xy = sigmaEllipse2D(mu, P);

%Now plot the generated points. You should see an elongated ellipse stretching from the top left corner to the bottom right. 
figure(1);
h1 = plot(xy(1,:), xy(2,:));
%Set the scale of x and y axis to be the same. This should be done if the two variables are in the same domain, e.g. both are measured in meters.
axis equal
hold on
%Also plot a star where the mean is, and make it have the same color as the ellipse.
plot(mu(1), mu(2), '*', 'color', h1.Color);

%% Problem 2:

%Here we give parameters for a Gaussian density. The parameter mu is the mean, and P is the covariance matrix.
mu = [-2; 1];
P = [4, -2; -2 2];

% Here we give the parameters for the affine transform
A = [4, -2; -2 2];
b = [-2; 1];

% Call your function
[mu_trans, Sigma_trans] = affineGaussianTransform(mu, P, A, b);

% Display your results
disp("mu_trans = ");
disp(mu_trans);
disp("Sigma_trans = ");
disp(Sigma_trans);

%% Problem 3

f = @(x)[x(2,:).*cos(x(1,:)); x(2,:).*sin(x(1,:))];

mu_x = [3;-2];
Sigma_x = [2, 0.3; 0.3, 1];

N = 10000;

[mu_y, Sigma_y, y_s] = approxGaussianTransform(mu_x, Sigma_x, f, N)

%% Problem 4

%Here we give parameters for a Gaussian density. The parameter mu_x is the 
%mean of the random variable,sigma2_x is the variance of the random 
%variable and sigma2_r is the variance of the of the measurement noise
mu_x = 5;
sigma2_x = 1;
sigma2_r = 9;

% Call your function
[mu, Sigma] = jointGaussian(mu_x, sigma2_x, sigma2_r);

% Display your results
disp("mu = ");
disp(mu);
disp("Sigma = ");
disp(Sigma);

%% Problem 5

%Here we give parameters for a Gaussian density. The parameter mu_x is the 
%mean of the random variable,sigma2_x is the variance of the random 
%variable and sigma2_r is the variance of the of the measurement noise
mu_x = 5;
sigma2_x = 1;
sigma2_r = 9;
y = 3;

% Call your function
[mu, sigma2] = posteriorGaussian(mu_x, sigma2_x, y, sigma2_r);

% Display your results
disp("mu = ");
disp(mu);
disp("Sigma = ");
disp(sigma2);

%% Problem 6

%Here we give parameters for a Gaussian density.
w = 1;
mu = 2;
s2 = 3;

% Call your function
[x_est] = gaussMixMMSEEst(w, mu, s2);

% Display your results
disp("x_est = ");
disp(x_est);