function X = genLinearStateSequence(x_0, P_0, A, Q, N)
%GENLINEARSTATESEQUENCE generates an N-long sequence of states using a 
%    Gaussian prior and a linear Gaussian process model
%
%Input:
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   A           [n x n] State transition matrix
%   Q           [n x n] Process noise covariance
%   N           [1 x 1] Number of states to generate
%
%Output:
%   X           [n x N+1] State vector sequence
%

X = zeros(size(x_0,1),N+1);                         % Initialize state vector sequence
initial_state = (mvnrnd(x_0,P_0))';                 % Draw initial state from prior distribution
q = (mvnrnd(zeros(size(Q,1),1),Q,N))';              % Draw samples from multidimensional Gaussian Distribution
X(:,1) = initial_state;                             % Set initial state
for iterator = 2:N+1
    X(:,iterator) = A*X(:,iterator - 1) + q(:,iterator - 1);    % Generate State Sequence
end

end