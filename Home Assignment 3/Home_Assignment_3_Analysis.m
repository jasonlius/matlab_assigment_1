%% SSY345: Sensor Fusion and Non-linear Filtering
% Home Assignment 3 (Analysis)
% 
% Author: Sourab Bapu Sridhar

clc
clear
close all

%% Problem 1: Approximations of mean and covariance

% User Selection
type = 'CKF';
state_density_type = 2;

% Prior
if state_density_type == 1
    mu_x = [120,120]';
    mu_Sigma = [5, 0; 0, 10].^2;
else
	mu_x = [120,-20]';
    mu_Sigma = [5, 0; 0, 10].^2;
end

% Sensor Position
s1 = [0,100]';
s2 = [100,0]';

% Measurement Noise
R = diag([0.1*pi/180 0.1*pi/180].^2);

% Measurement Model and Transformed Sequence
h = @(x)dualBearingMeasurement(x,s1,s2);
MeasurementSequence = @(x)genNonLinearMeasurementSequence(x, h, R);

% Problem 1.a: Generate Samples, Sample Mean and Sample Covariance
N = 1e5;
[mu_y, Sigma_y, y_s] = approxGaussianTransform(mu_x, mu_Sigma, MeasurementSequence, N);

% Problem 1.b: Compute three approximations of mean and covariance
[mu_ye, Sigma_ye] = approximateMeanAndCovariance(mu_x, mu_Sigma, h, R, type);

% Problem 1.c: Plot results
[xy1] = sigmaEllipse2D(mu_y, Sigma_y, 3, 50);
[xy2] = sigmaEllipse2D(mu_ye, Sigma_ye, 3, 50);

if (strcmp(type,'UKF')||strcmp(type,'CKF'))
    [SP,W] = sigmaPoints(mu_x, mu_Sigma, type);
    hSP = h(SP);
end

figure();
clf;
hold on
plot(y_s(1,:),y_s(2,:),'.g')
plot(xy1(1,:),xy1(2,:),'-r','Linewidth',2);
plot(xy2(1,:),xy2(2,:),'--b','Linewidth',2);
plot(mu_y(1,:),mu_y(2,:),'*r','Linewidth',4);
plot(mu_ye(1,:),mu_ye(2,:),'*k','Linewidth',2);
if (strcmp(type,'UKF')||strcmp(type,'CKF'))
    plot(hSP(1,:),hSP(2,:),'om','Linewidth',2);
end
xlabel('$\phi_{1}$','Interpreter','Latex');
ylabel('$\phi_{2}$','Interpreter','Latex');
title('Cubature Kalman Filter, State Density $p_{2}(x)$','Interpreter','Latex');
if (strcmp(type,'EKF'))
    legend('y = h(x) + r','Sample $3\sigma$ Value','Transformed $3\sigma$ Value','Sample Mean and Covariance','Approximate Mean and Covariance','Interpreter','Latex','Location','Southeast');
else
    legend('y = h(x) + r','Sample $3\sigma$ Value','Transformed $3\sigma$ Value','Sample Mean and Covariance','Approximate Mean and Covariance','Sigma Points','Interpreter','Latex','Location','Southeast');
end
hold off

% Problem 1.d: What differences can you see?
% Results available in the report

% Problem 1.e: Is it a good idea to apprximate the densoty p(y) with a
% Gaussian distribution?
% Results available in the report

%% Problem 2: Non-linear Kalman filtering
clc;
clear;
% User Selection
state_density_type = 1;

% Sampling Time
T = 1;

% Standard Deviation
sigma_v = 1;
sigma_w = (pi/180);
if state_density_type == 1
    sigma_phi1 = (10*pi/180);
else
    sigma_phi1 = (0.5*pi/180);
end
sigma_phi2 = (0.5*pi/180);

% Process and Measurement Noise
Q = diag([0, 0, T*sigma_v, 0, T*sigma_w].^2);
R = diag([sigma_phi1, sigma_phi2].^2);

% Prior
x_0 = [0, 0, 14, 0, 0]';
P_0 = diag([10, 10, 2, (pi/180), (5*pi/180)].^2);

% Sensor Position
s1 = [-200,100]';
s2 = [-200,-100]';

% Total Number of Samples
N = 100;

% Process and Measurement Model and Transformed Sequence
f = @(x)coordinatedTurnMotion(x,T);
X = genNonLinearStateSequence(x_0, P_0, f, Q, N);
h = @(x)dualBearingMeasurement(x,s1,s2);
Y = genNonLinearMeasurementSequence(X, h, R);

% Calculate Unfiltered Positions from Angles
Xm(1,:) = (s2(2)-s1(2)+tan(Y(1,:))*s1(1)- tan(Y(2,:))*s2(1))./(tan(Y(1,:))- tan(Y(2,:)));
Xm(2,:) = s1(2)+tan(Y(1,:)).*(Xm(1,:)- s1(1));

% Problem 2.a, 2.b: Plot Kalman Filter Results for Case 1 and 2
for type = {'EKF','UKF','CKF'}
    [xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, type{1});
        
    figure();
    clf;
    hold on
    plot(s1(1),s1(2),'*m','Linewidth',2);
    plot(s2(1),s2(2),'om','Linewidth',2);
    plot(X(1,:),X(2,:),'-k');
    plot(xf(1,:),xf(2,:),'-.b');
    plot(Xm(1,:),Xm(2,:),'+r');
	for i= 1:5:length(xf)
        [xy3] = sigmaEllipse2D(xf(1:2,i), Pf(1:2,1:2,i), 3, 50);
        plot(xy3(1,:),xy3(2,:),'--g');
	end
    xlabel('x');
    ylabel('y');
    title(sprintf("%s Filter, Case %d",type{1}, state_density_type));
    legend('Sensor 1 Position','Sensor 2 Position','True State','Filtered State','Measured State','$3\sigma$ region','Interpreter','Latex','Location','Southeast');
    hold off     
end

% Problem 2.c: Plot Histograms of Estimation Error
MC = 100;
type = {'EKF','UKF','CKF'};

est_err = cell(2,3);

for imc = 1:MC
    for state_density_type = 1:2
        
        x_0 = [0, 0, 14, 0, 0]';
        P_0 = diag([10, 10, 2, (pi/180), (5*pi/180)].^2);

        sigma_v = 1;
        sigma_w = (pi/180);
        if state_density_type == 1
            sigma_phi1 = (10*pi/180);
        else
            sigma_phi1 = (0.5*pi/180);
        end
        sigma_phi2 = (0.5*pi/180);
        
        T = 1;
        N = 100;
        
        Q = diag([0, 0, T*sigma_v, 0, T*sigma_w].^2);
        R = diag([sigma_phi1, sigma_phi2].^2);
        
        X = genNonLinearStateSequence(x_0, P_0, f, Q, N);
        Y = genNonLinearMeasurementSequence(X, h, R);
        
        for filter_type = 1:numel(type)
            [xf,Pf,xp,Pp] = nonLinearKalmanFilter(Y,x_0,P_0,f,Q,h,R,type{filter_type});
            est_err{state_density_type, filter_type}(1:2,end+1:end+length(xf)) = X(1:2,2:end)-xf(1:2,:);
        end
    end
end

bins = 100;
type = {'EKF','UKF','CKF'};
position = {'x','y'};

for state_density_type = 1:2
    for filter_type = 1:numel(type)
        for position_type = 1:numel(position)
            
            data = est_err{state_density_type,filter_type}(position_type,:);
            mu_data  = mean(data);
            sigma_data = std(data);
            number_of_points = 1e3;
            points = linspace(mu_data-5*sigma_data, mu_data+5*sigma_data, number_of_points);
            [gaussian_y] = normpdf(points,mu_data,sigma_data);
            
            figure();
            clf;
            hold on
            histogram(data,bins,'Normalization','pdf');
            plot(points,gaussian_y,'-r','Linewidth',2);            
            title(sprintf("%s Filter, Case %d, %s-Position Error\nMean = %f, Standard Deviation = %f",type{filter_type}, state_density_type, position{position_type}, mu_data, sigma_data));
            hold off  
        end
    end
end

%% Problem 3: Tuning non-linear filters
clc;
clear;
% Problem 3.a,3.b: Tune Q and evaluate the performance of the filter
sigma_v = 1 * 1e-4;
sigma_w = pi/180;

% True track
% Sampling period
T = 0.1;
% Length of time sequence
K = 600;
% Allocate memory
omega = zeros(1,K+1);
% Turn rate
omega(200:400) = -pi/201/T;
% Initial state
x0 = [0 0 20 0 omega(1)]';
% Allocate memory
X = zeros(length(x0),K+1);
X(:,1) = x0;
% Create true track
for i=2:K+1
    % Simulate
    X(:,i) = coordinatedTurnMotion(X(:,i-1), T);
    % Set turn-rate
    X(5,i) = omega(i);
end

x_0 = [0,0,0,0,0]';
P_0 = diag([10,10,10,(5*pi/180),(1*pi/180)].^2);

s1 = [280,-80]';
s2 = [280,-200]';

sigma_phi1 = (4*pi)/180;
sigma_phi2 = (4*pi)/180;

R = diag([sigma_phi1, sigma_phi2].^2);
h = @(x) dualBearingMeasurement(x,s1,s2);
Y = genNonLinearMeasurementSequence(X,h,R);

f = @(x) coordinatedTurnMotion(x,T);
Q = diag([0 0 T*sigma_v^2 0 T*sigma_w^2]);

[xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, 'CKF');

% Calcualte unfiltered position from sensors given angles
Xm(1,:) = (s2(2)-s1(2)+tan(Y(1,:))*s1(1)-tan(Y(2,:))*s2(1))./(tan(Y(1,:))- tan(Y(2,:)));
Xm(2,:) = s1(2)+tan(Y(1,:)).*(Xm(1,:)-s1(1));

% Problem 3.c: Generate Plots
figure();
clf;
hold on
plot(s1(1),s1(2),'*m','Linewidth',2);
plot(s2(1),s2(2),'om','Linewidth',2);
plot(X(1,:),X(2,:),'-k');
plot(xf(1,:),xf(2,:),'-.b');
plot(Xm(1,:),Xm(2,:),'+r');
for i= 1:10:length(xf)
	[xy4] = sigmaEllipse2D(xf(1:2,i), Pf(1:2,1:2,i), 3, 50);
	plot(xy4(1,:),xy4(2,:),'--g');
end
xlabel('x');
ylabel('y');
legend('Sensor 1 Position','Sensor 2 Position','True State','Filtered State','Measured State','$3\sigma$ region','Interpreter','Latex','Location','Southeast');
title("Tuning of Kalman Filter, Tuned Process Noise Case");
axis equal
hold off  

figure();
clf;
hold on
plot((1:K)*T,vecnorm(xf(1:2,:)-X(1:2,2:end),2,1));
xlabel('Time [s]');
ylabel('$||p_{k}-\hat{p}_{k|k}||_{2}$','Interpreter','Latex');
title("Tuning of Kalman Filter, Tuned Process Noise Case");
hold off 

% Problem 3.d: Is it possible to tune for the whole track?
% Results available in the report