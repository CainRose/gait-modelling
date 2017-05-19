function [answer] = new_bezier_order4(val_ini, val_fin, slope_ini, slope_fin, Fc, Time_vector)
% Constructs a 4th order Bezier curve with 5 given values: 
%                              Val_ini, val_fin, slope_ini, slope_fin, Fc

% Calculate End time
Tfinal = Time_vector(end);
% Calculate 5 bezier parameters for 4th order Bezier function
bezier_param = [val_ini;((slope_ini*Tfinal)+(4*val_ini))/4;0;((4*val_fin)-(slope_fin*Tfinal))/4;val_fin];
bezier_param(3) = ((16*Fc)-(bezier_param(1))-(bezier_param(5))-(4*bezier_param(2))-(4*bezier_param(4)))/6;
% Construct Bezier trajectory
for i = 1:length(Time_vector)
    Z(i) = ((1/Tfinal)^4)*((bezier_param(1)*((Tfinal - Time_vector(i))^4))+((4*bezier_param(2))*(((Tfinal-Time_vector(i))^3)*(Time_vector(i))))+((6*bezier_param(3))*(((Tfinal-Time_vector(i))^2)*((Time_vector(i))^2)))+((4*bezier_param(4))*(((Tfinal-Time_vector(i)))*((Time_vector(i))^3)))+(bezier_param(5)*((Time_vector(i))^4)));     
end
% keyboard();

answer = Z;
end

