%% SSY345: Sensor Fusion and Non-linear Filtering
% Home Assignment 3 (Implementation)
% 
% Author: Sourab Bapu Sridhar

clc
clear
close all

%% Problem 1: coordinatedTurnMotion

% Call your function
[fx, Fx] = coordinatedTurnMotion([3; 2; 1; pi/2; pi/4], 0.1);

% Display your results
disp("fx =");
disp(fx);
disp("Fx =");
disp(Fx);

%% Problem 2: dualBearingMeasurement

% Here we give the general parameters for the Measurement Model
x = rand(6,1);
s1 = rand(2,1);
s2 = rand(2,1);

% Call your function
[hx, Hx] = dualBearingMeasurement(x, s1, s2);

% Display your results
disp("hx =");
disp(hx);
disp("Hx =");
disp(Hx);

%% Problem 3: genNonLinearStateSequence

% Number of time steps;
N = 500;

% Define prior
x_0     = rand(5,1); 
n       = length(x_0); 
P_0     = rand(5,5);
P_0     = P_0*P_0';

% Sample time
T = 1;

% Covariance
sigV = 1;
sigOmega = 1*pi/180;
G = [zeros(2,2); 1 0; 0 0; 0 1];
Q = G*diag([sigV^2 sigOmega^2])*G';

% Motion model function handle. Note how sample time T is inserted into the function.
motionModel = @(x) coordinatedTurnMotion(x, T);

% generate state sequence
X = genNonLinearStateSequence(x_0, P_0, motionModel, Q, N);

% show some results
disp("X =");
disp(X(:,1:5));

%% Problem 4: genNonLinearMeasurementSequence

% Number of states
N = 500;

% Position sequence
X = [(1:N); rand(1,N)];

% Random sensor position sequence
s1 = [0, 100]';
s2 = [0, -100]';

% Measurement noise covariance
R = diag([2*pi/180 2*pi/180].^2);

% Measurement model
measModel = @(X) dualBearingMeasurement(X, s1, s2);

% Generate measurements
Y = genNonLinearMeasurementSequence(X, measModel, R);

% Show some results
disp("Y =");
disp(Y(:,1:5));

%% Problem 5: sigmaPoints

% Here we give general parameters for calculating sigma points
x = rand(2,1);
P = rand(2,2);

% Call your function
[SP,W] = sigmaPoints(x, P, 'UKF');

% Display your results
disp("Sigma Points for UKF =");
disp(SP);
disp("Weights for UKF =");
disp(W);

% Call your function
[SP,W] = sigmaPoints(x, P, 'CKF');

% Display your results
disp("Sigma Points for CKF =");
disp(SP);
disp("Weights for CKF =");
disp(W);

%% Problem 6: nonLinKFprediction

% Here we give general parameters
x = rand(5,1);
P = rand(5,5);
T = 1;
f = @(x)coordinatedTurnMotion(x, T);
Q = rand(5,5);

% Call your function
[x, P] = nonLinKFprediction(x, P, f, Q, 'EKF');

% Display your results
disp("Predicted Mean for EKF =");
disp(x);
disp("Predicted Covariance for EKF =");
disp(P);

% Call your function
[x, P] = nonLinKFprediction(x, P, f, Q, 'UKF');

% Display your results
disp("Predicted Mean for UKF =");
disp(x);
disp("Predicted Covariance for UKF =");
disp(P);

% Call your function
[x, P] = nonLinKFprediction(x, P, f, Q, 'CKF');

% Display your results
disp("Predicted Mean for CKF =");
disp(x);
disp("Predicted Covariance for CKF =");
disp(P);

%% Problem 7: nonLinKFupdate

% Here we give general parameters
x = rand(5,1);
P = sqrt(10)*rand(5,5);
P = P*P';
s1 = rand(2,1)*100;
s2 = rand(2,1)*100;
h = @(x) dualBearingMeasurement(x, s1, s2);
y = h(x)+rand(2,1);
R = diag(rand(1,2).^2);

% Call your function
[x, P] = nonLinKFupdate(x, P, y, h, R, 'EKF');

% Display your results
disp("Updated Mean for EKF =");
disp(x);
disp("Updated Covariance for EKF =");
disp(P);

% Call your function
[x, P] = nonLinKFupdate(x, P, y, h, R, 'UKF');

% Display your results
disp("Updated Mean for UKF =");
disp(x);
disp("Updated Covariance for UKF =");
disp(P);

% Call your function
[x, P] = nonLinKFupdate(x, P, y, h, R, 'CKF');

% Display your results
disp("Updated Mean for CKF =");
disp(x);
disp("Updated Covariance for CKF =");
disp(P);

%% Problem 8: nonLinearKalmanFilter

% Here we give general parameters for Kalman Filter
x_0 = rand(5,1);
P_0 = sqrt(10)*rand(5,5);
P_0 = P_0*P_0';
T = 1;
f = @(x)coordinatedTurnMotion(x, T);
Q = diag(rand(1,5).^2);
s1 = rand(2,1)*100;
s2 = rand(2,1)*100;
h = @(x) dualBearingMeasurement(x, s1, s2);
Y = h(x_0)+rand(2,1);
R = diag(rand(1,2).^2);

% Call your function
[xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, 'EKF');

% Display your results
disp("Filtered Mean for EKF =");
disp(xf);
disp("Filtered Covariance for EKF =");
disp(Pf);
disp("Predicted Mean for EKF =");
disp(xp);
disp("Predicted Covariance for EKF =");
disp(Pp);

% Call your function
[xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, 'UKF');

% Display your results
disp("Filtered Mean for UKF =");
disp(xf);
disp("Filtered Covariance for UKF =");
disp(Pf);
disp("Predicted Mean for UKF =");
disp(xp);
disp("Predicted Covariance for UKF =");
disp(Pp);

% Call your function
[xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, 'CKF');

% Display your results
disp("Filtered Mean for CKF =");
disp(xf);
disp("Filtered Covariance for CKF =");
disp(Pf);
disp("Predicted Mean for CKF =");
disp(xp);
disp("Predicted Covariance for CKF =");
disp(Pp);