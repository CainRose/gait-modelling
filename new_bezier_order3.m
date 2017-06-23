function [answer] = new_bezier_order3(val_ini, val_fin, slope_ini, slope_fin, Time_vector)
% Constructs a 3rd order Bezier curve with 4 given values: 
%                              Val_ini, val_fin, slope_ini, slope_fin

% Curve is in the form [a*t^3 + b*t^2(tf-t) + c*t(tf-t)^2 + d*(tf-t)^3]/tf^3 

Tf = Time_vector(end);
param = [   val_ini, ...
            val_ini + slope_ini*Tf/3, ...
            val_fin - slope_fin*Tf/3, ...
            val_fin     ];

b = @(t) (  param(1)*(Tf-t).^3 + 3 * param(2)*t.*(Tf-t).^2 + ... 
            3 * param(3)*t.^2.*(Tf-t) + param(4)*t.^3  ) / Tf^3;

answer = b(Time_vector);

end

