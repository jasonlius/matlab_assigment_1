%% SSY345: Sensor Fusion and Non-linear Filtering
% Home Assignment 2 (Analysis)
% 
% Author: Sourab Bapu Sridhar

clc
clear
close all

%% Problem 1: A first Kalman filter and its properties

A = 1;
H = 1;
Q = 1.5;
R = 2.5;
x_0 = 2;
P_0 = 6;
N = 20;

% Problem 1.a
X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);

figure();
plot(X(2:end),'-r');
hold on
plot(Y, '*b');
xlabel('Sequence');
ylabel('Samples');
title('State Sequence and Measurement Sequence for A = 1 and H = 1');
legend('State Sequence', 'Measurement Sequence','location','northeast');
hold off

% Problem 1.b
[x, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);

figure(); 
clf; 
hold on;
plot([1:N], X(2:end), 'g');
plot([1:N], Y, '*r');
plot([0:N], [x_0 x], 'b');
plot([0:N], [x_0 x] + 3*sqrt([P_0 P(:)']), '--b');
plot([0:N], [x_0 x] - 3*sqrt([P_0 P(:)']), '--b');
xlabel('k');
ylabel('x');
title('Kalman Filter Output')
legend('state sequence','measurements', 'state estimate', '+3-sigma level', '-3-sigma level','Location','southeast');
hold off

points_to_consider = [3, 8, 13, 18];

for iterator = 1:length(points_to_consider)
    instances = linspace(-15,15,1000);
    posterior_density = normpdf(instances, x(:,iterator), P(:,:,iterator));
    true_state = normpdf(instances, X(:,iterator+1), Q);

    figure();
    clf;
    hold on;
    plot(instances, posterior_density, 'b','linewidth', 2);
    plot([X(:,iterator+1) X(:,iterator+1)], [0 max(posterior_density)], '--r','linewidth', 2);
    xlabel('k');
    ylabel('x');
    title('Posterior Density and True State')
    legend('Posterior Density','True State','Location','northeast');
    hold off;
end


% Problem 1.c
instances = linspace(-15,15,1000);
prior = normpdf(instances, X(:,10), Q);
[xPred, PPred] = linearPrediction(X(:,10), P(:,:,10), A, Q);
prediction = normpdf(instances, xPred, PPred);
measurement = normpdf(instances, Y(:,10), R);
posterior_density = normpdf(instances, x(:,10), P(:,:,10));

figure();
clf;
hold on;
plot(instances, prior, '-b','linewidth',2);
plot(instances, prediction, '*b');
plot([Y(:,10) Y(:,10)], [0 max(posterior_density)], '--g','linewidth', 2);
plot([X(:,11) X(:,11)], [0 max(posterior_density)], '--m','linewidth', 2);
plot(instances, posterior_density, '-r','linewidth',2);
xlabel('k');
ylabel('x');
title('Various Probablity Densities of Kalman Filter')
legend('$P(x_{9}|y_{1:9})$', '$P(x_{10}|y_{1:9})$', '$y_{10}$', '$x_{10}$', '$P(x_{10}|y_{1:10})$','Interpreter','latex','Location','northeast');
hold off;

% Problem 1.d
N = 1000;
instances = linspace(-15,15,1000);
X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);
[x, P, v] = kalmanFilter2(Y, x_0, P_0, A, Q, H, R);

error_estimation = (X(2:end) - x);
error_estimation_pdf = normpdf(instances, 0, P(:,:,end));

figure();
clf;
hold on;
histogram(error_estimation,'Normalization','pdf');
plot(instances,error_estimation_pdf,'-r','linewidth',2);
xlabel('k');
ylabel('x');
title('Histogram of the estimation error')
legend('$histogram(x_{k}-\hat{x_{k}})$','$\mathcal{N}(x_{0},0,P_{N|N})$','Interpreter','latex','location','Northeast')
hold off

figure();
clf;
hold on;
autocorr(v);
xlabel('Lag');
ylabel('Autocorelation');
title('Autocorelation of innovation')
legend('Autocorelation','location','Northeast')
hold off

% Problem 1.f
N = 20;
x_0_CorrectPrior = 2;
P_0_CorrectPrior = 6;

X = genLinearStateSequence(x_0_CorrectPrior, P_0_CorrectPrior, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);

[x_CorrectPrior, P_CorrectPrior] = kalmanFilter(Y, x_0_CorrectPrior, P_0_CorrectPrior, A, Q, H, R);

x_0_IncorrectPrior = 10;
P_0_IncorrectPrior = 6;
[x_IncorrectPrior, P_IncorrectPrior] = kalmanFilter(Y, x_0_IncorrectPrior, P_0_IncorrectPrior, A, Q, H, R);

figure(); 
clf; 
hold on;
plot([1:N], X(2:end), 'k','Linewidth',2);
plot([1:N], Y, '*m');
plot([0:N], [x_0_CorrectPrior x_CorrectPrior], 'b','Linewidth',2);
plot([0:N], [x_0_CorrectPrior x_CorrectPrior] + 3*sqrt([P_0_CorrectPrior P_CorrectPrior(:)']), '--b','Linewidth',2);
plot([0:N], [x_0_CorrectPrior x_CorrectPrior] - 3*sqrt([P_0_CorrectPrior P_CorrectPrior(:)']), '--b','Linewidth',2);

plot([0:N], [x_0_IncorrectPrior x_IncorrectPrior], 'r','Linewidth',2);
plot([0:N], [x_0_IncorrectPrior x_IncorrectPrior] + 3*sqrt([P_0_IncorrectPrior P_IncorrectPrior(:)']), '--r','Linewidth',2);
plot([0:N], [x_0_IncorrectPrior x_IncorrectPrior] - 3*sqrt([P_0_IncorrectPrior P_IncorrectPrior(:)']), '--r','Linewidth',2);
xlabel('k');
ylabel('x');
title('Kalman Filter Output')
legend('True State','Measurements', 'State estimate', '+3-sigma level', '-3-sigma level', 'State estimate', '+3-sigma level', '-3-sigma level', 'Location','southeast');
hold off

%% Problem 2: Kalman filter and its tuning
T = 0.01;
A = [1, T; 0, 1];
H = [1, 0];
Q = [0,0;0,1.5];
R = 2;
x_0 = [1; 3];
P_0 = 4*eye(2);
N = 50;

% Problem 2.a
X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);

figure();
plot(X(1,2:end),'-r');
hold on
plot(Y(1,:), '*b');
xlabel('Time Step');
ylabel('Position');
title('Position State Sequence and Position Measurement Sequence');
legend('Position State Sequence', 'Position Measurement Sequence','location','northeast');
hold off

figure();
plot(X(2,2:end),'-r');
xlabel('Time Step');
ylabel('Velocity');
title('Velocity State Sequence');
legend('Velocity State Sequence','location','northeast');
hold off

% Problem 2.b
[x, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);

figure(); 
clf; 
hold on;
plot([1:N], X(1,2:end), 'g');
plot([1:N], Y(1,:), '*r');
plot([0:N], [x_0(1) x(1,:)], 'b');
plot([0:N], [x_0(1) x(1,:)] + 3*sqrt([P_0(1) squeeze(P(1,1,:))']), '--b');
plot([0:N], [x_0(1) x(1,:)] - 3*sqrt([P_0(1) squeeze(P(1,1,:))']), '--b');
xlabel('k');
ylabel('x');
title('Kalman Filter Output')
legend('True State','Measurement', 'State estimate', '+3-sigma level', '-3-sigma level','Location','southeast');
hold off

figure(); 
clf; 
hold on;
plot([1:N], X(2,2:end), 'g');
plot([0:N], [x_0(2) x(2,:)], 'b');
plot([0:N], [x_0(2) x(2,:)] + 3*sqrt([P_0(2) squeeze(P(2,2,:))']), '--b');
plot([0:N], [x_0(2) x(2,:)] - 3*sqrt([P_0(2) squeeze(P(2,2,:))']), '--b');
xlabel('k');
ylabel('v');
title('Kalman Filter Output')
legend('True State','State estimate', '+3-sigma level', '-3-sigma level','Location','southeast');
hold off

% Problem 2.c
Q = [0,0;0,2];
R = 2;

X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);
    
points_to_consider = [0.1, 1, 10, 1.5];

for iterator = 1:length(points_to_consider)
    
    Q(2,2) = points_to_consider(iterator);
    
    [x, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);

    figure(); 
    clf; 
    hold on;
    plot([1:N], X(1,2:end), 'g');
    plot([1:N], Y(1,:), '*r');
    plot([0:N], [x_0(1) x(1,:)], 'b');
    plot([0:N], [x_0(1) x(1,:)] + 3*sqrt([P_0(1) squeeze(P(1,1,:))']), '--b');
    plot([0:N], [x_0(1) x(1,:)] - 3*sqrt([P_0(1) squeeze(P(1,1,:))']), '--b');
    xlabel('k');
    ylabel('x');
    title('Kalman Filter Output')
    legend('True State','Measurement', 'State estimate', '+3-sigma level', '-3-sigma level','Location','southeast');
    hold off
end