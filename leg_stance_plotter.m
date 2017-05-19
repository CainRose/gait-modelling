function [x, y, z] = leg_stance_plotter(DH_mat, q)
x = zeros(1,4);
y = zeros(1,4);
z = zeros(1,4);
% keyboard();
DH_mat(:,4) = q;
x(1) = 0; y(1) = 0; z(1) = 0;
T = transformation_mat(DH_mat(1,:));
x(2) = T(1,4); y(2) = T(2,4); z(2) = T(3,4);
T = T*transformation_mat(DH_mat(2,:));
x(3) = T(1,4); y(3) = T(2,4); z(3) = T(3,4);
T = T*transformation_mat(DH_mat(3,:));
x(4) = T(1,4); y(4) = T(2,4); z(4) = T(3,4);
end

