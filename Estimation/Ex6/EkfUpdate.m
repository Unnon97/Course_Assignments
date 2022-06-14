function [xPosterior,PPosterior]= EkfUpdate(xPrior,PPrior,z,R,radarState)
% Task 5 - Complete this function
    zk = [((xPrior(1)-radarState(1))^2 + (xPrior(2)-radarState(2))^2)^0.5 ; atan2( (xPrior(2)-radarState(2)) , (xPrior(1)-radarState(1)) ) ];
    HH = MeasurementMatrix(xPrior,radarState);
    S = HH*PPrior*HH' + R;
    W = (PPrior*HH')/S;
    xPosterior = xPrior + W*(z-zk);
    PPosterior = PPrior - W*S*W';
end