%% SSY345: Sensor Fusion and Non-linear Filtering
% Home Assignment 1 (Analysis)
% 
% Author: Sourab Bapu Sridhar

clc
clear
close all
%% Problem 1: Transformation of Gaussian random variables
mu_x = [10;0];
Sigma_x = [0.2,0;0,0.8];

A = [1,1;1,-1];
b = [0;0];

LinearTransform = @(x)[A*x];

[mu_affine_linear, Sigma_affine_linear] = affineGaussianTransform(mu_x, Sigma_x, A, b);
[mu_approx_linear, Sigma_approx_linear, Gaussian_approx_linear] = approxGaussianTransform(mu_x, Sigma_x, LinearTransform, 5000);

xy_affine_3_sigma_linear = sigmaEllipse2D(mu_affine_linear, Sigma_affine_linear, 3);
xy_approx_3_sigma_linear = sigmaEllipse2D(mu_approx_linear, Sigma_approx_linear, 3);

figure;
plot(xy_affine_3_sigma_linear(1,:), xy_affine_3_sigma_linear(2,:),'color','blue');
axis equal
hold on
plot(xy_approx_3_sigma_linear(1,:), xy_approx_3_sigma_linear(2,:),'color','red');
axis equal
hold on
plot(mu_affine_linear(1),mu_affine_linear(2),'*','color','black','LineWidth',2);
axis equal
hold on
plot(mu_approx_linear(1),mu_approx_linear(2),'*','color','black','LineWidth',2);
axis equal
hold on
plot(Gaussian_approx_linear(1,:),Gaussian_approx_linear(2,:),'.','color','cyan');
axis equal
hold on
legend('3\sigma_{affine}','3\sigma_{approx}','mean_{affine}','mean_{approx}','Samples','location','northeast');
hold off

NonlinearTransform = @(x)[sqrt(x(1,:).^2 + x(2,:).^2);atan2(x(2,:),x(1,:))];

[mu_approx_nonlinear, Sigma_approx_nonlinear, Gaussian_approx_nonlinear] = approxGaussianTransform(mu_x, Sigma_x, NonlinearTransform, 5000);

xy_approx_3_sigma_nonlinear = sigmaEllipse2D(mu_approx_nonlinear, Sigma_approx_nonlinear, 3);

figure;
plot(xy_approx_3_sigma_nonlinear(1,:), xy_approx_3_sigma_nonlinear(2,:),'color','red');
axis equal
hold on 
plot(mu_approx_nonlinear(1),mu_approx_nonlinear(2),'*','color','black','LineWidth',2);
axis equal
hold on 
plot(Gaussian_approx_nonlinear(1,:),Gaussian_approx_nonlinear(2,:),'.','color','cyan');
axis equal
hold on
legend('3\sigma_{approx}','mean_{approx}','Samples','location','northeast')
hold off

%% Problem 2: Snow depth in Norway

mu_Hafjell = 1.1;
mu_Kvitfjell = 1.0;
Sigma2_snow = 0.5^2;

y_Hafjell = 1;
y_Kvitfjell = 2;

mu_Anna = 0;
mu_Else = 0;
Sigma2_Anna = 0.2^2;
Sigma2_Else = 1^2;

[mu_xy_Hafjell, Sigma_xy_Hafjell] = jointGaussian(mu_Hafjell, Sigma2_snow, Sigma2_Anna);
[mu_xy_Kvitfjell, Sigma_xy_Kvitfjell] = jointGaussian(mu_Kvitfjell, Sigma2_snow, Sigma2_Else);

xy_Hafjell = sigmaEllipse2D(mu_xy_Hafjell, Sigma_xy_Hafjell, 3);
xy_Kvitfjell = sigmaEllipse2D(mu_xy_Kvitfjell, Sigma_xy_Kvitfjell, 3);

figure;
plot(xy_Hafjell(1,:), xy_Hafjell(2,:));
axis equal
hold on
plot(xy_Kvitfjell(1,:), xy_Kvitfjell(2,:));
axis equal
hold on
legend('3\sigma_{Hafjell}', '3\sigma_{Kvitfjell}')
hold off

[mu_x_Hafjell_y_Anna, sigma2_x_Hafjell_y_Anna] = posteriorGaussian(mu_Hafjell, Sigma2_snow, y_Hafjell, Sigma2_Anna);
[mu_x_Kvitfjell_y_Else, sigma2_x_Kvitfjell_y_Else] = posteriorGaussian(mu_Kvitfjell, Sigma2_snow, y_Kvitfjell, Sigma2_Else);

x = -5:0.01:5;
x_Hafjell_y_Anna = normpdf(x, mu_x_Hafjell_y_Anna, sqrt(sigma2_x_Hafjell_y_Anna));
x_Kvitfjell_y_Else = normpdf(x, mu_x_Kvitfjell_y_Else, sqrt(sigma2_x_Kvitfjell_y_Else));

figure;
plot(x, x_Hafjell_y_Anna);
hold on
plot(x, x_Kvitfjell_y_Else);
hold on
legend('p(x_{H}|y_{A})', 'p(x_{K}|y_{E})')
hold off

%% Problem 3: MMSE and MAP estimates for Gaussian mix-ture posteriors

x = -15:0.01:15;

w_3a = [0.1, 0.9];
mu_3a = [1, 1];
Sigma2_3a = [0.5, 9];

pdf_3a = w_3a(1)*normpdf(x, mu_3a(1), sqrt(Sigma2_3a(1))) + w_3a(2)*normpdf(x, mu_3a(2), sqrt(Sigma2_3a(2)));
mmse_3a = gaussMixMMSEEst(w_3a, mu_3a, Sigma2_3a);
[max_3a, index_max_3a] = max(pdf_3a);
map_3a = x(index_max_3a);
figure;
plot(x, pdf_3a);
hold on
plot(mmse_3a, 0, 'o')
hold on
plot(map_3a, 0, 'x')
hold on
legend('Posterior', 'MMSE', 'MAP')
hold off

w_3b = [0.49, 0.51];
mu_3b = [5, -5];
Sigma2_3b = [2, 2];

pdf_3b = w_3b(1)*normpdf(x, mu_3b(1), sqrt(Sigma2_3b(1))) + w_3b(2)*normpdf(x, mu_3b(2), sqrt(Sigma2_3b(2)));
mmse_3b = gaussMixMMSEEst(w_3b, mu_3b, Sigma2_3b);
[max_3b, index_max_3b] = max(pdf_3b);
map_3b = x(index_max_3b);
figure;
plot(x, pdf_3b);
hold on
plot(mmse_3b, 0, 'o')
hold on
plot(map_3b, 0, 'x')
hold on
legend('Posterior', 'MMSE', 'MAP')
hold off

w_3c = [0.4, 0.6];
mu_3c = [1, 2];
Sigma2_3c = [2, 1];

pdf_3c = w_3c(1)*normpdf(x, mu_3c(1), sqrt(Sigma2_3c(1))) + w_3c(2)*normpdf(x, mu_3c(2), sqrt(Sigma2_3c(2)));
mmse_3c = gaussMixMMSEEst(w_3c, mu_3c, Sigma2_3c);
[max_3c, index_max_3c] = max(pdf_3c);
map_3c = x(index_max_3c);
figure;
plot(x, pdf_3c);
hold on
plot(mmse_3c, 0, 'o')
hold on
plot(map_3c, 0, 'x')
hold on
legend('Posterior', 'MMSE', 'MAP')
hold off