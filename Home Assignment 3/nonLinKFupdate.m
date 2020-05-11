function [x, P] = nonLinKFupdate(x, P, y, h, R, type)
%NONLINKFUPDATE calculates mean and covariance of predicted state
%   density using a non-linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   y           [m x 1] measurement vector
%   h           Measurement model function handle
%               [hx,Hx]=h(x) 
%               Takes as input x (state), 
%               Returns hx and Hx, measurement model and Jacobian evaluated at x
%               Function must include all model parameters for the particular model, 
%               such as sensor position for some models.
%   R           [m x m] Measurement noise covariance
%   type        String that specifies the type of non-linear filter
%
%Output:
%   x           [n x 1] updated state mean
%   P           [n x n] updated state covariance
%

    switch type
        case 'EKF'
            
            % Your EKF update here
            [hx,Hx] = h(x);            
            S = Hx*P*Hx' + R;                                           % Calculate Innovation Covariance
            K = P*Hx'*inv(S);                                           % Calculate Innovation Gain
            x = x + K*(y - hx);                                         % Updated Mean
            P = P - K*S*K';                                             % Updated Covariance
        
        case 'UKF'
    
            % Your UKF update here
            [SP,W] = sigmaPoints(x, P, type);
            yhat = zeros(size(y));            
            for i = 1:size(W,2)
                yhat = yhat + h(SP(:,i))*W(i);                          % Calculate Measurements
            end
            
            Pxy = zeros(size(P,1),size(R,1));            
            for i = 1:size(W,2)
                Pxy = Pxy + (SP(:,i) - x)*(h(SP(:,i)) - yhat)'*W(i);    % Calculate Cross-Covariance
            end

            S = R;
            for i = 1:size(W,2)
                S = S + (h(SP(:,i)) - yhat)*(h(SP(:,i)) - yhat)'*W(i);  % Calculate Innovation Covariance
            end
            
            x = x + Pxy*inv(S)*(y - yhat);                              % Updated Mean
            P = P - Pxy*inv(S)*Pxy';                                    % Updated Covariance
    
            % Make sure the covariance matrix is semi-definite
            if min(eig(P))<=0
                [v,e] = eig(P, 'vector');
                e(e<0) = 1e-4;
                P = v*diag(e)/v;
            end
            
        case 'CKF'
    
            % Your CKF update here
            [SP,W] = sigmaPoints(x, P, type);
            yhat = zeros(size(y));            
            for i = 1:size(W,2)
                yhat = yhat + h(SP(:,i))*W(i);                          % Calculate Measurements
            end
            
            Pxy = zeros(size(P,1),size(R,1));            
            for i = 1:size(W,2)
                Pxy = Pxy + (SP(:,i) - x)*(h(SP(:,i)) - yhat)'*W(i);    % Calculate Cross-Covariance
            end

            S = R;
            for i = 1:size(W,2)
                S = S + (h(SP(:,i)) - yhat)*(h(SP(:,i)) - yhat)'*W(i);  % Calculate Innovation Covariance
            end
            
            x = x + Pxy*inv(S)*(y - yhat);                              % Updated Mean
            P = P - Pxy*inv(S)*Pxy';                                    % Updated Covariance
    
            % Make sure the covariance matrix is semi-definite
            if min(eig(P))<=0
                [v,e] = eig(P, 'vector');
                e(e<0) = 1e-4;
                P = v*diag(e)/v;
            end
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end

end

