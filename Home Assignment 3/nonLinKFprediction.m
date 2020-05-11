function [x, P] = nonLinKFprediction(x, P, f, Q, type)
%NONLINKFPREDICTION calculates mean and covariance of predicted state
%   density using a non-linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   f           Motion model function handle
%               [fx,Fx]=f(x) 
%               Takes as input x (state), 
%               Returns fx and Fx, motion model and Jacobian evaluated at x
%               All other model parameters, such as sample time T,
%               must be included in the function
%   Q           [n x n] Process noise covariance
%   type        String that specifies the type of non-linear filter
%
%Output:
%   x           [n x 1] predicted state mean
%   P           [n x n] predicted state covariance
%

    n = size(x,1);
    switch type
        case 'EKF'
            
            % Your EKF code here
            [fx,Fx] = f(x);                 
            x = fx;                                                 % Calculate Predicted Mean
            P = Fx*P*Fx' + Q;                                       % Calculate Predicted Covariance
            
        case 'UKF'
    
            % Your UKF code here
            [SP,W] = sigmaPoints(x, P, type);                       % Calculate Sigma Points
            x = zeros(n,1);                                         % Initialize Predicted Mean
            for i = 1:size(W,2)
                x = x + f(SP(:,i))*W(i);                            % Calculate Predicted Mean
            end
            
            P = Q;                                                  % Initialize Predicted Covariance
            for i = 1:size(W,2)
                P = P + (f(SP(:,i)) - x)*(f(SP(:,i)) - x).'*W(i);   % Calculate Predicted Covariance
            end
            
            % Make sure the covariance matrix is semi-definite
            if min(eig(P))<=0
                [v,e] = eig(P, 'vector');
                e(e<0) = 1e-4;
                P = v*diag(e)/v;
            end
                
        case 'CKF'
            
            % Your CKF code here
            [SP,W] = sigmaPoints(x, P, type);                       % Calculate Sigma Points
            x = zeros(n,1);                                         % Initialize Predicted Mean
            for i = 1:size(W,2)
                x = x + f(SP(:,i))*W(i);                            % Calculate Predicted Mean
            end
            
            P = Q;                                                  % Initialize Predicted Covariance
            for i = 1:size(W,2)
                P = P + (f(SP(:,i)) - x)*(f(SP(:,i)) - x).'*W(i);   % Calculate Predicted Covariance
            end
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end

end