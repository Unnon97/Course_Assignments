function [measMatrix]= MeasurementMatrix(state,radarState)
% Task 4 - Complete this function
    measMatrix = [ (state(1)-radarState(1)) / (((state(1)-radarState(1))^2 + (state(2)-radarState(2))^2)^0.5), (state(2)-radarState(2)) / (((state(1)-radarState(1))^2 + (state(2)-radarState(2))^2)^0.5), 0, 0;
                   ((-state(2)+radarState(2))/((state(2)-radarState(2))^2 + (state(1)-radarState(1))^2)), ((state(1)-radarState(1))/((state(2)-radarState(2))^2 + (state(1)-radarState(1))^2)),0 ,0 ]; 
    
end