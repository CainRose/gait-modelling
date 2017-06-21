function [bz, bpitch] = select_toes(xFR, zFR, xFL, zFL, xHR, zHR, xHL, zHL, IGD)
t1x = xFR;
t1z = zFR;
t2x = xHR;
t2z = zHR;

delta_x = IGD + t1x - t2x;
delta_z = +t1z - t2z;

temp_pitch = delta_z/delta_x; %(global pitch value)
% keyboard();
check_z1 = t1z + temp_pitch*(xFL - t1x);
if abs(zFL) >= abs(check_z1)
    t1x = xFL;
    t1z = zFL;
end
% keyboard();
check_z2 = t2z + temp_pitch*(xHL - t2x);
if abs(zHL) >= abs(check_z2)
    t2x = xHL;
    t2z = zHL;
end
% keyboard();
% at this stage, t1 and t2 are correctly selected...
delta_x = IGD + t1x - t2x;
delta_z = t2z - t1z; % t2 and t1 here are flipped... this is correct

bpitch = asin(delta_z/delta_x);
rotation_mat = [cos(-bpitch) sin(-bpitch);-sin(-bpitch) cos(-bpitch)];

world_frame_hind_coordinates = rotation_mat*[t2x;t2z];
world_frame_front_coordinates = rotation_mat*[t1x;t1z];

bz = (abs(world_frame_hind_coordinates(2))+abs(world_frame_front_coordinates(2)))/2;
% keyboard();

end

