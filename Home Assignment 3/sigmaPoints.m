function [SP,W] = sigmaPoints(x, P, type)
% SIGMAPOINTS computes sigma points, either using unscented transform or
% using cubature.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%
%Output:
%   SP          [n x 2n+1] UKF, [n x 2n] CKF. Matrix with sigma points
%   W           [1 x 2n+1] UKF, [1 x 2n] UKF. Vector with sigma point weights 
%

    n = size(x,1);
    switch type        
        case 'UKF'
    
            SP = zeros(n,2*n+1);                                    % Initialize Sigma Points            
            W0 = 1-n/3;                                             % Initialize Weights
            SP(:,1) = x;                                            % Initialize First Sigma Point
            SqrtP = sqrtm(P);
            
            for i = 1:size(P,2)
                SP(:,i+1) = x + sqrt((n)/(1-W0))*SqrtP(:,i);        % Calculate Remaining Sigma Points
                SP(:,i+n+1) = x - sqrt((n)/(1-W0))*SqrtP(:,i);      % Calculate Remaining Sigma Points
            end
            
            W = [W0, ((1-W0)/(2*n))*ones(1,2*n)];                   % Calculate Remaining Weights
                
        case 'CKF'
            
            SP = zeros(n,2*n);                                      % Initialize Sigma Points
            SqrtP = sqrtm(P);
            
            for i = 1:size(P,2)
                SP(:,i) = x + sqrt(n)*SqrtP(:,i);                   % Calculate Sigma Points
                SP(:,i+n) = x - sqrt(n)*SqrtP(:,i);                 % Calculate Sigma Points
            end
            
            W = ((1)/(2*n))*ones(1,2*n);                            % Calculate Weights
            
        otherwise
            error('Incorrect type of sigma point')
    end
end