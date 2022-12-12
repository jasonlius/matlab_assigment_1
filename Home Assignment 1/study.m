clc
clear
close all
%%Problem 1:高斯随机变量的转换
%写出原来X的二维高斯分布
mu_x = [10;0];
Sigma_x = [0.2,0;0,0.8];
%写出Y的转换系数
A = [1,1;1,-1];
b = [0;0];
LinearTransform = @(x)[A*x];
%调用函数计算转换后的函数的高斯分布
[mu_affine_linear, Sigma_affine_linear] = affineGaussianTransform(mu_x, Sigma_x, A, b);
[mu_approx_linear, Sigma_approx_linear, Gaussian_approx_linear] = approxGaussianTransform(mu_x, Sigma_x, LinearTransform, 5000);

xy_affine_3_sigma_linear = sigmaEllipse2D(mu_affine_linear, Sigma_affine_linear, 3)
xy_approx_3_sigma_linear = sigmaEllipse2D(mu_approx_linear, Sigma_approx_linear, 3)

f = figure("Name","对比图",NumberTitle="off");
plot(xy_affine_3_sigma_linear(1,:), xy_affine_3_sigma_linear(2,:),"Color","black","LineWidth",2)
axis equal
hold on
plot(xy_approx_3_sigma_linear(1,:), xy_approx_3_sigma_linear(2,:),"Color","red","LineWidth",2)
axis equal
hold on
scatter(Gaussian_approx_linear(1,:), Gaussian_approx_linear(2,:),10,"cyan","filled")
axis equal
hold on
plot(mu_affine_linear(1), mu_affine_linear(2),'*',"Color","black","LineWidth",2);
axis equal
hold on
plot(mu_approx_linear(1), mu_approx_linear(2),'*',"Color","black","LineWidth",2);
axis equal
hold on
legend('3\sigma_{affine}',"3\sigma_{approx}",'Samples','mean_{affine}','mean_{approx}','location','northeast')
hold off

NonlinearTransform = @(x)[sqrt(x(1,:).^2 + x(2,:).^2);atan2(x(2,:),x(1,:))];

[mu_approx_nonlinear, Sigma_approx_nonlinear, Gaussian_approx_nonlinear] = approxGaussianTransform(mu_x, Sigma_x, NonlinearTransform, 5000);

xy_approx_3_sigma_nonlinear = sigmaEllipse2D(mu_approx_nonlinear, Sigma_approx_nonlinear, 3);

figure;
plot(xy_approx_3_sigma_nonlinear(1,:), xy_approx_3_sigma_nonlinear(2,:),'color','red','LineWidth',2);
axis equal
hold on 
plot(mu_approx_nonlinear(1),mu_approx_nonlinear(2),'*','color','black','LineWidth',2);
axis equal
hold on 
plot(Gaussian_approx_nonlinear(1,:),Gaussian_approx_nonlinear(2,:),'.','color','cyan','LineWidth',2);
axis equal
hold on
legend('3\sigma_{approx}','mean_{approx}','Samples','location','northeast')
hold off

% %%Problem 2: 挪威雪的深度
Hafjell_mu_x = 1.1;
Hafjell_sigma2_x = 0.5.^2;
Anna_sigma2_r = 0.2.^2;
[Hafjell_mu, Hafjell_Sigma]= jointGaussian(Hafjell_mu_x, Hafjell_sigma2_x, Anna_sigma2_r);

mu_x = 1.0;
sigma2_x = 0.5.^2;
sigma2_r = 1.^2;
[mu, Sigma]= jointGaussian(mu_x, sigma2_x, sigma2_r);

xy_Hafjell_3_sigma = sigmaEllipse2D(Hafjell_mu, Hafjell_Sigma, 3)
xy_sigma = sigmaEllipse2D(mu, Sigma, 3)
data_points = mvnrnd(Hafjell_mu, Hafjell_Sigma, 5000);
data_point2s = mvnrnd(mu, Sigma, 5000);
figure;
plot(xy_Hafjell_3_sigma(1,:), xy_Hafjell_3_sigma(2,:),"Color","blue","LineWidth",2)
axis equal
hold on
plot(data_points(:,1),data_points(:,2),".","Color","red","LineWidth",2)
axis equal
hold on
plot(xy_sigma(1,:), xy_sigma(2,:),"Color","blue","LineWidth",2)
axis equal
hold on
plot(data_point2s(:,1),data_point2s(:,2),".","Color","black","LineWidth",2)
axis equal
hold off












