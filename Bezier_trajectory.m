function [theta, dtheta, ddtheta] = Bezier_trajectory( Number_Parameters, thetaInit, dthetaInit, thetaFinal, dthetaFinal, Extra_Parameters, T, Steps)

timeStep = T/Steps;
% Steps = floor(T/timeStep);

theta_time = zeros(1,Steps+1);
theta_values = zeros(1, Steps+1);
dtheta_values = zeros(1, Steps+1);
ddtheta_values = zeros(1, Steps+1);
%1) first must construct the bezier parameters.
    Spacing = 1/(Number_Parameters + 4 - 1);
        Vector(1) = thetaInit;
        Vector(2) = ...
            (Spacing*dthetaInit*T + thetaInit);
        Vector(3:(3 + Number_Parameters - 1)) = Extra_Parameters;
        Vector(3 + Number_Parameters) =...
            ( - dthetaFinal*Spacing*T + thetaFinal);
        Vector(4 + Number_Parameters) = thetaFinal;
%% Construct trajectory
        M = 1/Spacing;
        for counter = 1:Steps+1
% %             counter
            t = (counter-1)/Steps; %%normalized time
            theta_time(counter) = (counter-1)*timeStep;    
            for i = 0:(M)
                P1 = factorial(M)/(factorial(i)*factorial(M - i));
                theta_values(counter)   = theta_values(counter)   + P1*(t^i)*(1-t)^(M-i)*Vector(i+1);
                dtheta_values(counter)  = dtheta_values(counter)  + (Vector(i+1)*P1*(t^(i - 1)*(i - M*t)*(1 - t)^(M - i - 1)))/T;
                ddtheta_values(counter) = ddtheta_values(counter) + (Vector(i+1)*P1*(-t^(i - 2)*(1 - t)^(M - i - 2)*(- i^2 + 2*i*M*t - 2*i*t + i - M^2*t^2 + M*t^2)))/(T^2);  
            end   
        end
%% remove last value for dtheta and ddtheta since they are NAN
% keyboard();
%        dtheta_values(1) =  dthetaInit;
%        ddtheta_values(1) = ddtheta_values(2);      
%        dtheta_values(Steps+1) =  dthetaFinal;
%        ddtheta_values(Steps+1) =  ddtheta_values(Steps);       

theta = vertcat(theta_time, theta_values);
dtheta = vertcat(theta_time, dtheta_values);
ddtheta = vertcat(theta_time, ddtheta_values);

end


