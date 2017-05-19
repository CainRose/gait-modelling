function [x, y, z, rot_mat] = forward_kin(DH_table)
% Calculates the end-effector position, given the DH-table for the
% manipulator arm/leg

% AEP front leg
leg_transform = (transformation_mat(DH_table(1,:)))*(transformation_mat(DH_table(2,:)))*(transformation_mat(DH_table(3,:)));
x = leg_transform(1,4);
y = leg_transform(2,4);
z = leg_transform(3,4);
rot_mat = leg_transform(1:3, 1:3);

end

