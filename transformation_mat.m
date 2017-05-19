function [T] = transformation_mat(v)
%Transformation matrix
T(1, 1) = cos(v(4));
T(1, 2) = -cos(v(2))*sin(v(4));
T(1, 3) = sin(v(2))*sin(v(4));
T(1, 4) = v(1)*cos(v(4));
T(2, 1) = sin(v(4));
T(2, 2) = cos(v(2))*cos(v(4));
T(2, 3) = -cos(v(4))*sin(v(2));
T(2, 4) = v(1)*sin(v(4));
T(3, 1) = 0;
T(3, 2) = sin(v(2));
T(3, 3) = cos(v(2));
T(3, 4) = v(3);
T(4, 1) = 0;
T(4, 2) = 0;
T(4, 3) = 0;
T(4, 4) = 1;
end

