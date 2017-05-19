function [answer] = new_bezier_order3(val_ini, val_fin, slope_ini, slope_fin, Time_vector)
% Constructs a 3rd order Bezier curve with 4 given values: 
%                              Val_ini, val_fin, slope_ini, slope_fin

% Calculate End time
Tfinal = Time_vector(end);
% Calculate 4 bezier parameters for 3rd order Bezier function
bezier_param = [val_ini;((slope_ini*Tfinal)+(3*val_ini))/3;((3*val_fin)-(slope_fin*Tfinal))/3;val_fin];

% Construct Bezier trajectory
for i = 1:length(Time_vector)
    Z(i) = ((1/Tfinal)^3)*((bezier_param(1)*((Tfinal - Time_vector(i))^3))+((3*bezier_param(2))*(((Tfinal-Time_vector(i))^2)*(Time_vector(i))))+((3*bezier_param(3))*((Tfinal-Time_vector(i))*((Time_vector(i))^2)))+(bezier_param(4)*((Time_vector(i))^3)));     
end
% keyboard();

answer = Z;
end

