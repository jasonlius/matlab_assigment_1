function X = genNonLinearStateSequence(x_0, P_0, f, Q, N)
%GENNONLINEARSTATESEQUENCE generates an N+1-long sequence of states using a 
%    Gaussian prior and a nonlinear Gaussian process model
%
%Input:
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   f           Motion model function handle
%               [fx,Fx]=f(x) 
%               Takes as input x (state), 
%               Returns fx and Fx, motion model and Jacobian evaluated at x
%               All other model parameters, such as sample time T,
%               must be included in the function
%   Q           [n x n] Process noise covariance
%   N           [1 x 1] Number of states to generate
%
%Output:
%   X           [n x N+1] State vector sequence
%

% Your code here
X = zeros(size(x_0,1),N+1);                         % Initialize state vector sequence
initial_state = (mvnrnd(x_0,P_0))';                 % Draw initial state from prior distribution
q = (mvnrnd(zeros(size(Q,1),1),Q,N))';              % Draw samples from multidimensional Gaussian Distribution
X(:,1) = initial_state;                             % Set initial state
for iterator = 2:N+1
    [fx,~] = f(X(:,iterator - 1));                  % Generate States
    X(:,iterator) = fx + q(:,iterator - 1);         % Generate State Sequence
end
end