%% SSY345: Sensor Fusion and Non-linear Filtering
% Home Assignment 2 (Implementation)
% 
% Author: Sourab Bapu Sridhar

clc
clear
close all
%% Problem 1: genLinearStateSequence

%Here we give parameters for a Gaussian density. The parameter mu is the 
%mean, and P is the covariance matrix.
mu = [-2; 1];
P = [4, -2; -2, 2];

%Here we give the parameters for the affine transform
A = [4, -2; -2, 2];

%Here we give parameters for the noise covariance.
Q = [1, 0; 0, 9];

%Here we give parameters for the number of states to generate.
N = 10;

% Call your function
X = genLinearStateSequence(mu, P, A, Q, N);

% Display your results
disp("X = ");
disp(X);

%% Problem 2: genLinearMeasurementSequence

%Here we give parameters for a state sequence X
X = [ 1,2, 3,4, 5,6, 5,4, 3, 2,1;
     -1,2,-3,4,-5,6,-7,8,-9,10,9];

%Here we give the parameters for the measurement model
H = [4, -2; -2, 2];

%Here we give parameters for the noise covariance.
R = [1, 0; 0, 9];

% Call your function
Y = genLinearMeasurementSequence(X, H, R);

% Display your results
disp("Y = ");
disp(Y);

%% Problem 3: linearPrediction

% Call your function
[x, P] = linearPrediction(1, 1, 1, 1);

% Display your results
disp("x = ");
disp(x);
disp("P = ");
disp(P);

%% Problem 4: linearUpdate

% Call your function
[x, P] = linearUpdate(1, 1, 2, 1, 1);

% Display your results
disp("x = ");
disp(x);
disp("P = ");
disp(P);

%% Problem 5: Kalman Filter

% Here we give general parameters for Kalman Filter
N = 10;
n = 2;
m = 1;
T = [1,0;0,9];

% Here we give the Motion Parameters
A = [1,2;-2,-1];
Q = T*.5^2;

% Here we give the Measurement Parameters
H = [1,3];
R = 1^2;

% Here we give the Prior
xPrior  = [1;1];
PPrior  = [2,0;0,5].^2;

% Here we genereate the Measurement Sequence
measuremetnSequence = 10*ones(m,N);

% Call your function
[stateSequence, covarianceSequence] = kalmanFilter(measuremetnSequence, xPrior, PPrior, A, Q, H, R);

% Display your results
disp("stateSequence = ");
disp(stateSequence);
disp("covarianceSequence = ");
disp(covarianceSequence);