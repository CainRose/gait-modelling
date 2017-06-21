function [bz, bpitch] = select_toes(xFR, zFR, xFL, zFL, xHR, zHR, xHL, zHL, IGD)

% Arbitrarily select left most toe as back ground contact and calculate angles to other toes
if (xHL < xHR); t2x = xHL;  t2z = zHL;
else            t2x = xHR;  t2z = zHR;
end

angles = [  atan((zFL - t2z)/(xFL + IGD - t2x)), ...
            atan((zFR - t2z)/(xFR + IGD - t2x)), ...
            atan((zHL - t2z)/(xHL - t2x)),       ...
            atan((zHR - t2z)/(xHR - t2x))       ];

% find leg with smallest angle (most negative)
[leg_pitch, i] = min(angles);

% If this is a hind leg, recalculate angles and change back leg
if (i > 2)
    if (i == 3);    t2x = xHL;  t2z = zHL;
    else            t2x = xHR;  t2z = zHR;
    end
    angles = [  atan((zFL - t2z)/(xFL + IGD - t2x)), ...
                atan((zFR - t2z)/(xFR + IGD - t2x)) ];
    [leg_pitch, i] = min(angles);
end
if (i == 1) % Front Left
    t1x = xFL;  t1z = zFL;
else        % Front Right
    t1x = xFR;  t1z = zFR;
end

bpitch = -leg_pitch;    % To counteract the rotation of the leg

rotation_mat = [cos(-bpitch) sin(-bpitch);-sin(-bpitch) cos(-bpitch)];

world_frame_hind_coordinates = rotation_mat*[t2x;t2z];
world_frame_front_coordinates = rotation_mat*[t1x;t1z];

bz = (abs(world_frame_hind_coordinates(2))+abs(world_frame_front_coordinates(2)))/2;
% keyboard();

end

