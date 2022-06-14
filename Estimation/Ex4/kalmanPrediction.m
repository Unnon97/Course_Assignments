function [priorMean,priorCovariance] = kalmanPrediction(posteriorMean,posteriorCovar,F,Q)
    %TASK 3 - Complete this function 
    priorMean =  F *posteriorMean ;
    priorCovariance = F*posteriorCovar*F.' + Q;
end

