function [xPosterior,PPosterior]= UkfUpdate(xPrior,PPrior,z,R,radarState)
% Task 6 - Complete this function#
    kappa = 0;
    samplSize = 2*length(xPrior) + 1;
    w = ones(1,samplSize)*(1/(samplSize+kappa));
    w(1) = 0;
    w = w./sum(w);
    sample = zeros(length(xPrior),samplSize);
    hk = zeros(2,samplSize);
    for i = 1:samplSize
        extraElement = real(sqrtm((length(xPrior)+kappa)*PPrior))';
        if i==1
            sample(:,i) = xPrior;
        end
        if i <= (samplSize+1)/2 && i > 1
            sample(:,i) = xPrior + extraElement(:,i-1);
        end
        if i> (samplSize+1)/2
            sample(:,i) = xPrior - extraElement(:,i-length(xPrior)-1);
        end
        hk(:,i) = [((sample(1,i)-radarState(1))^2 + (sample(2,i)-radarState(2))^2)^0.5 ; atan2( sample(2,i)-radarState(2) , sample(1,i)-radarState(1) ) ];
    end
    X = sample;
    xk = xPrior;
    zkk = sum(hk.*w,2);
    Pzz = zeros(2,2);
    for i = 1:samplSize
        ele = (hk(:,i)-zkk)*(hk(:,i)-zkk)';
        Pzz = Pzz + ele.*w(i);
    end
    Pxz = zeros(4,2);
    for i = 1:samplSize
        elem = (X(:,i)-xk)*(hk(:,i)-zkk)';
        Pxz = Pxz + elem.*w(i);
    end
    Sk = R + Pzz;
    Kk = Pxz/Sk;
    xPosterior = xPrior + Kk*(z-zkk);
    PPosterior = PPrior - Kk*Sk*Kk';
end