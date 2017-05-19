function [ power_loss ] = at4_power_loss( stride_length, time, torquefl1, torquefr1, torquehl1, torquehr1,  torquefl2, torquefr2, torquehl2, torquehr2, torquefl3, torquefr3, torquehl3, torquehr3, Tmax )
%POWER_LOSS Summary of this function goes here
% Equation (32)
%   Detailed explanation goes here

        integrand = (torquefl1.*torquefl1)+(torquefr1.*torquefr1)+(torquehl1.*torquehl1)+(torquehr1.*torquehr1)+(torquefl2.*torquefl2)+(torquefr2.*torquefr2)+(torquehl2.*torquehl2)+(torquehr2.*torquehr2)+(torquefl3.*torquefl3)+(torquefr3.*torquefr3)+(torquehl3.*torquehl3)+(torquehr3.*torquehr3);
        integrandf1 = trapz(time, integrand);

        % Note that the total time in the data series might exceed Tmax
        % The equation below factors this accordingly by scaling it with Tmax/TTotal
        power_loss = (1/stride_length)*(Tmax/time(end))*(integrandf1);
   
end

