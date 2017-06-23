function [answer] = new_bezier_order4(val_ini, val_fin, slope_ini, slope_fin, Fc, Time_vector)
% Constructs a 4th order Bezier curve with 5 given values: 
%                              Val_ini, val_fin, slope_ini, slope_fin, Fc

Tf = Time_vector(end);
param = [   val_ini, ...
            val_ini + slope_ini*Tf/3, ...
            0 , ...
            val_fin - slope_fin*Tf/3, ...
            val_fin     ];
param(3) = (16*Fc - param(1) - 4*param(2) - 4*param(4) - param(5)) / 6;

b = @(t) (  param(1) * (Tf-t).^4   +   4*param(2) * t.*(Tf-t).^3   + ... 
            6*param(3) * t.^2.*(Tf-t).^2   +   4*param(4) * t.^3.*(Tf-t)  + ...
            param(5) * t.^4)   /   Tf^4;

        
answer = b(Time_vector);
% % Calculate End time
% Tfinal = Time_vector(end);
% % Calculate 5 bezier parameters for 4th order Bezier function
% bezier_param = [val_ini;((slope_ini*Tfinal)+(4*val_ini))/4;0;((4*val_fin)-(slope_fin*Tfinal))/4;val_fin];
% bezier_param(3) = ((16*Fc)-(bezier_param(1))-(bezier_param(5))-(4*bezier_param(2))-(4*bezier_param(4)))/6;
% % Construct Bezier trajectory
% for i = 1:length(Time_vector)
%     Z(i) = ((1/Tfinal)^4)*((bezier_param(1)*((Tfinal - Time_vector(i))^4))+((4*bezier_param(2))*(((Tfinal-Time_vector(i))^3)*(Time_vector(i))))+((6*bezier_param(3))*(((Tfinal-Time_vector(i))^2)*((Time_vector(i))^2)))+((4*bezier_param(4))*(((Tfinal-Time_vector(i)))*((Time_vector(i))^3)))+(bezier_param(5)*((Time_vector(i))^4)));     
% end
% % keyboard();
% 
% answer = Z;
end

