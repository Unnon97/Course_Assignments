function [posteriorMean,posteriorCovariance,gainX] = kalmanUpdate(priorMean,priorCovariance,z,H,R)
    % TASK 3 - Complete this function
    K = priorCovariance * H.' /(H*priorCovariance*H.' + R) ;
    posteriorMean = priorMean + K *(z - H*priorMean);
    posteriorCovariance = (eye(4) - K *H)*priorCovariance;
    gainX = K(1,1);
end

