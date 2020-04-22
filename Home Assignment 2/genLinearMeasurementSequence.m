function Y = genLinearMeasurementSequence(X, H, R)
%GENLINEARMEASUREMENTSEQUENCE generates a sequence of observations of the state 
% sequence X using a linear measurement model. Measurement noise is assumed to be 
% zero mean and Gaussian.
%
%Input:
%   X           [n x N+1] State vector sequence. The k:th state vector is X(:,k+1)
%   H           [m x n] Measurement matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   Y           [m x N] Measurement sequence
%

Y = zeros(size(H,1),size(X,2)-1);                       % Initialize observation sequence
r = (mvnrnd(zeros(size(R,1),1),R,size(X,2)-1))';        % Draw measurement noise from multidimensional Gaussian Distribution
for iterator = 1:(size(X,2)-1)
    Y(:,iterator) = H*X(:,iterator+1)+r(:,iterator);    % Generate observation sequence
end
end