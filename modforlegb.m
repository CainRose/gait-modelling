function [ ansp_mod_11, ansp_mod_22 ] = modforlegb(ansp, phi_rel, T_final, obj_ite_stride_time)
% keyboard();
[~, length_ansp] = size(ansp);
length_should_be = floor((obj_ite_stride_time/0.001) + 1);

% keyboard();
    diff = (length_should_be - length_ansp);
    if diff
        for i = 1:diff
            ansp = [ansp, ansp(:,end)];
        end
    end
%    keyboard();
    midpoint_index = floor(length_should_be*(1-phi_rel));
%     keyboard();
    ansp_mod_seca = ansp(:,1:midpoint_index);
    ansp_mod_secb = ansp(:,(midpoint_index+1):(end));
%     keyboard();
    ansp_mod_1 = [ansp_mod_secb, ansp_mod_seca];
%     keyboard();
    midpoint_index = floor(length_should_be*0.5);
% keyboard();    
    ansp_mod_secc = ansp_mod_1(:,1:midpoint_index);
    ansp_mod_secd = ansp_mod_1(:,(midpoint_index+1):(end));
%     keyboard();
    ansp_mod_2 = [ansp_mod_secd, ansp_mod_secc];
%     keyboard();
    total_adds = floor(T_final/obj_ite_stride_time);
%     keyboard();
    temp2 = ansp_mod_2;
    temp1 = ansp_mod_1;
    for i = 1:total_adds
        ansp_mod_1 = [ansp_mod_1, temp1];
        ansp_mod_2 = [ansp_mod_2, temp2];
    end
%   keyboard();  
    ansp_mod_11 = ansp_mod_1(:, 1:5001);
    ansp_mod_22 = ansp_mod_2(:, 1:5001);
%     keyboard();
end

