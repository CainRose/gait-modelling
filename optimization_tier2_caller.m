warning('off','all');
tic;
clc
% keyboard();
global obj_ite_velocity obj_ite_stride_length obj_ite_duty_factor obj_ite_phi_rel_fb obj_ite_stride_time obj_ite_simulation_time obj_ite_stride_height obj_ite_step_clearance obj_ite_step_length obj_ite_take_off_angle obj_ite_angle_change obj_ite_OF_scale

global lg1ps lg2ps lg3ps lg4ps simulation_time trans_leg_1_rot bx by bz density densityw density_body body_height gravity k_g_v n b_g_v p q S_p fk IGD
global wr1 wl1 wr2 wl2 wr3 wl3 wr4 wl4 wheel_radius obj_ite_max_torque obj_ite_max_torque_change velocity_initial pitch_initial
global lg1kp lg2kp lg3kp lg4kp lg1ki lg2ki lg3ki lg4ki lg1kd lg2kd lg3kd lg4kd lg1kb lg2kb lg3kb lg4kb body_height_calc
global M1 M2 M3 M4 L1 L2 L3 L4 l1 l2 l3 l4 R1 R2 R3 R4 r1 r2 r3 r4
global TRJ_LG1_J1 TRJ_LG1_J2 TRJ_LG1_J3 TRJ_LG1_J4 TRJ_LG2_J1 TRJ_LG2_J2 TRJ_LG2_J3 TRJ_LG2_J4 TRJ_LG3_J1 TRJ_LG3_J2 TRJ_LG3_J3 TRJ_LG3_J4 TRJ_LG4_J1 TRJ_LG4_J2 TRJ_LG4_J3 TRJ_LG4_J4 TRJ_LG1_w TRJ_LG2_w TRJ_LG3_w TRJ_LG4_w
global TRJ_LG1_J1_TIMESERIES TRJ_LG1_J2_TIMESERIES TRJ_LG1_J3_TIMESERIES TRJ_LG2_J1_TIMESERIES TRJ_LG2_J2_TIMESERIES TRJ_LG2_J3_TIMESERIES TRJ_LG3_J1_TIMESERIES TRJ_LG3_J2_TIMESERIES TRJ_LG3_J3_TIMESERIES TRJ_LG4_J1_TIMESERIES TRJ_LG4_J2_TIMESERIES TRJ_LG4_J3_TIMESERIES
global AEP_L1_J1 AEP_L1_J2 AEP_L1_J3 AEP_L1_J4 AEP_L2_J1 AEP_L2_J2 AEP_L2_J3 AEP_L2_J4 AEP_L3_J1 AEP_L3_J2 AEP_L3_J3 AEP_L3_J4 AEP_L4_J1 AEP_L4_J2 AEP_L4_J3 AEP_L4_J4
global KPFLsw1 KPFLsw2 KPFLsw3 KPFLst1 KPFLst2 KPFLst3 KDFLsw1 KDFLsw2 KDFLsw3 KDFLst1 KDFLst2 KDFLst3
global KPFRsw1 KPFRsw2 KPFRsw3 KPFRst1 KPFRst2 KPFRst3 KDFRsw1 KDFRsw2 KDFRsw3 KDFRst1 KDFRst2 KDFRst3
global KPHLsw1 KPHLsw2 KPHLsw3 KPHLst1 KPHLst2 KPHLst3 KDHLsw1 KDHLsw2 KDHLsw3 KDHLst1 KDHLst2 KDHLst3
global KPHRsw1 KPHRsw2 KPHRsw3 KPHRst1 KPHRst2 KPHRst3 KDHRsw1 KDHRsw2 KDHRsw3 KDHRst1 KDHRst2 KDHRst3
global body_pitch_max body_roll_max body_yaw_max
global front_x front_y back_x back_y

body_pitch_max = pi/8;
body_roll_max = pi/6; 
body_yaw_max = pi/6;
global ground_x ground_y ground_z
ground_x = 24;
ground_y = 4;
ground_z = 0.1;
scale = 1;
HLL = scale*0.33;
FLL = scale*0.33;
IGD = scale*0.33;
comb = [0;body_yaw_max;body_pitch_max;body_roll_max];
save('body_movement_max.mat', 'comb');
% simulation_time = obj_ite_simulation_time;
trans_leg_1_rot = [1 0 0; 0 1 0; 0 0 1];
bx =IGD; by =0.5*0.5*IGD; bz =0.25*IGD;
front_x = IGD/2;      
back_x = -IGD/2; 
obj_ite_max_torque = 50;
obj_ite_max_torque_change = 50;
density = 1408;
densityw = 500;
density_body = 2700;
gravity = -9.81;
k_g_v = 7.21e7; n = 2.31;   b_g_v = 3.8e4;  p = 1.1;    q = 1;  S_p = 0.001;    fk = 0.6;
% k_g_v = 7.21e7; n = 2.31;   b_g_v = 3.8e6;  p = 1.1;    q = 1;  S_p = 0.001;    fk = 1.0;

% obj_ite_velocity = 0.1;
% obj_ite_simulation_time = 5;   
% obj_ite_OF_scale = [10^(2) 10^(2) 10^(-1) 10^(-2) 10^(-1) 10^(2) 10^(1)];
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%           DH PARAMETERS SET UP AND TRANSFORMATION MATRICES
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

lg1ps = [FLL*0.43, 0, 0, 0; ...
         FLL*0.40, 0, 0, 0;...
         FLL*0.17, 0, 0, 0];
lg2ps = [FLL*0.43, 0, 0, 0; ...
         FLL*0.40, 0, 0, 0;...
         FLL*0.17, 0, 0, 0];
lg3ps = [FLL*0.39, 0, 0, 0; ...
         FLL*0.36, 0, 0, 0;...
         FLL*0.25, 0, 0, 0];
lg4ps = [FLL*0.39, 0, 0, 0; ...
         FLL*0.36, 0, 0, 0;...
         FLL*0.25, 0, 0, 0];     
     
for i = 1:3
	t_mat_lg1(:,:,i) = transformation_mat(lg1ps(i,:));     
    t_mat_lg2(:,:,i) = transformation_mat(lg2ps(i,:));     
    t_mat_lg3(:,:,i) = transformation_mat(lg3ps(i,:));     
    t_mat_lg4(:,:,i) = transformation_mat(lg4ps(i,:));              
end

link_setup

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%           CONTROLLER PARAMETERS SET UP
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

obj_ite_velocity = 1;
velocity_initial = obj_ite_velocity;
                
design_var_lower = [0.1;...
                    0;...
                    0;...  
                    0;...
                    0.1;...  LEG FL beta
                    -1.9;... AEP FL J1 
                    0.7;...  AEP FL J2
                    0.4;... AEP FL J3
                    -0.1;...      delta Z = -10% of step length
                    -pi/2;...   delta psi 
                    0.1;...     Fc = 10% of step length 
                    0;...    z_dot_AEP
                    -0.1*IGD;...    z_dot_PEP
                    -pi/2;...   psi_dot_AEP
                    -pi/2;...   psi_dot_PEP
                    100;...     Kp_sw_j1_FL
                    1;...       Kd_sw_j1_FL
                    100;...
                    1;...
                    100;...
                    1;...
                    100;...     Kp_st_j1_FL
                    1;...       Kd_sw_j1_FL
                    100;...
                    1;...
                    100;...
                    1;...
                    0.1;...  LEG FR beta
                    -1.9;... AEP FR J1 
                    0.7;...  AEP FR J2
                    0.4;... AEP FR J3
                    -0.1;...      delta Z = -100% of step length
                    -pi/2;...   delta psi 
                    0.1;...     Fc = 10% of step length 
                    0;...    z_dot_AEP
                    -0.1*IGD;...    z_dot_PEP
                    -pi/2;...   psi_dot_AEP
                    -pi/2;...   psi_dot_PEP
                    100;...     Kp_sw_j1_FR
                    1;...       Kd_sw_j1_FR
                    100;...
                    1;...
                    100;...
                    1;...
                    100;...     Kp_st_j1_FR
                    1;...       Kd_sw_j1_FR
                    100;...
                    1;...
                    100;...
                    1;...
                    0.1;...  LEG HL beta
                    -1.0;...    AEP HL J1
                    -1.4;...    AEP HL J2
                     1;...      AEP HL J3
                    -0.1;...      delta Z = -100% of step length
                    -pi/2;...   delta psi 
                    0.1;...     Fc = 10% of step length 
                    0;...    z_dot_AEP
                    -0.1*IGD;...    z_dot_PEP
                    -pi/2;...   psi_dot_AEP
                    -pi/2;...   psi_dot_PEP
                    100;...     Kp_sw_j1_HL
                    1;...       Kd_sw_j1_HL
                    100;...
                    1;...
                    100;...
                    1;...
                    100;...     Kp_st_j1_HL
                    1;...       Kd_sw_j1_HL
                    100;...
                    1;...
                    100;...
                    1;...
                    0.1;...  LEG HR beta
                    -1.0;...    AEP HR J1
                    -1.4;...    AEP HR J2
                     1;...      AEP HR J3
                    -0.1;...      delta Z = -100% of step length
                    -pi/2;...   delta psi 
                    0.1;...     Fc = 10% of step length 
                    0;...    z_dot_AEP
                    -0.1*IGD;...    z_dot_PEP
                    -pi/2;...   psi_dot_AEP
                    -pi/2;...   psi_dot_PEP
                    100;...     Kp_sw_j1_HR
                    1;...       Kd_sw_j1_HR
                    100;...
                    1;...
                    100;...
                    1;...
                    100;...     Kp_st_j1_HR
                    1;...       Kd_sw_j1_HR
                    100;...
                    1;...
                    100;...
                    1];
                    
 design_var_upper = [2*IGD;...
                    1;...
                    1;...  
                    1;...
                    0.8;...  LEG FL beta
                    -1.6;... 
                    1.1;...
                    0.7;...
                    0.1;...      delta Z = -100% of step length
                    pi/2;...   delta psi 
                    0.5;...     Fc = 10% of step length 
                    0.1*IGD;...    z_dot_AEP
                    0;...    z_dot_PEP
                    pi/2;...   psi_dot_AEP
                    pi/2;...   psi_dot_PEP
                    1000;...     Kp_sw_j1_FL
                    10;...       Kd_sw_j1_FL
                    1000;...
                    10;...
                    1000;...
                    10;...
                    1000;...     Kp_st_j1_FL
                    10;...       Kd_sw_j1_FL
                    1000;...
                    10;...
                    1000;...
                    10;...
                    0.8;...  LEG FR beta
                    -1.6;... 
                    1.1;...
                    0.7;...
                    0.1;...      delta Z = -100% of step length
                    pi/2;...   delta psi 
                    0.5;...     Fc = 10% of step length 
                    0.1*IGD;...    z_dot_AEP
                    0;...    z_dot_PEP
                    pi/2;...   psi_dot_AEP
                    pi/2;...   psi_dot_PEP
                    1000;...     Kp_sw_j1_FR
                    10;...       Kd_sw_j1_FR
                    1000;...
                    10;...
                    1000;...
                    10;...
                    1000;...     Kp_st_j1_FR
                    10;...       Kd_sw_j1_FR
                    1000;...
                    10;...
                    1000;...
                    10;...
                    0.8;...  LEG HL beta
                    -0.6;...
                    -0.8;...
                    1.4;...
                    0.1;...      delta Z = -100% of step length
                    pi/2;...   delta psi 
                    0.5;...     Fc = 10% of step length 
                    0.1*IGD;...    z_dot_AEP
                    0;...    z_dot_PEP
                    pi/2;...   psi_dot_AEP
                    pi/2;...   psi_dot_PEP
                    1000;...     Kp_sw_j1_HL
                    10;...       Kd_sw_j1_HL
                    1000;...
                    10;...
                    1000;...
                    10;...
                    1000;...     Kp_st_j1_HL
                    10;...       Kd_sw_j1_HL
                    1000;...
                    10;...
                    1000;...
                    10;...
                    0.8;...  LEG HR beta
                    -0.6;...
                    -0.8;...
                    1.4;...
                    0.1;...      delta Z = -100% of step length
                    pi/2;...   delta psi 
                    0.5;...     Fc = 10% of step length 
                    0.1*IGD;...    z_dot_AEP
                    0;...    z_dot_PEP
                    pi/2;...   psi_dot_AEP
                    pi/2;...   psi_dot_PEP
                    1000;...     Kp_sw_j1_HR
                    10;...       Kd_sw_j1_HR
                    1000;...
                    10;...
                    1000;...
                    10;...
                    1000;...     Kp_st_j1_HR
                    10;...       Kd_sw_j1_HR
                    1000;...
                    10;...
                    1000;...
                    10];                   
                
% design_var = [-2.1095	0.842961	0.624199	-1.15244	-0.847325	1.27901	0.13294	0.0034745	8.66074e-06	0.166281	0.00377665	0	0.300528	0.0837145	0.0215225	0.633738	-1.681	0.315484	0.277376	0.780375	0.630281	262.595	709.383	743.533	100	379.386	957.87	9.46961	4.22589	1.84608	5.68886	4.62113	5.33759];
% design_var = [0.3   0.5 0.75	0.25	0.6	-1.8095	0.942961	0.624199	0	0.4	0.5	0	0	0.1	0.1	300	3	300	3	300	3	300	3	300	3	300	3	0.6	-1.8095	0.942961	0.624199	0	0.4	0.5	0	0	0.1	0.1	300	3	300	3	300	3	300	3	300	3	300	3	0.6	-0.85244	-0.947325	1.27901	0	0.4	0.5	0	0   0.1	0.1	300	3	300	3	300	3	300	3	300	3	300	3	0.6	-0.85244	-0.947325	1.27901	0	0.4	0.5	0	0	0.1	0.1	300	3	300	3	300	3	300	3	300	3	300	3];
% design_var = [0.507854	0.918221	0.441417	0.334112	0.312885	-1.63657	0.905201	0.56483	0.0907042	0.52268	0.276502	0.0142599	-0.0199586	1.36436	1.20659	840.437	9.10655	335.41	2.7286	559.595	5.50103	275.908	3.79931	464.608	9.75341	887.61	7.90159	0.573127	-1.69564	1.06173	0.476031	0.00864355	1.37817	0.341433	0.00764411	-0.015144	-0.591587	-1.03212	719.717	8.50953	240.475	8.44061	317.826	5.03113	529.919	5.97007	433.308	8.05859	855.243	1.79572	0.781666	-0.605418	-0.892911	1.13837	0.0968093	0.384308	0.416867	0.00126129	-0.0202903	0.395342	-1.18455	899.389	1.33276	908.996	7.73169	788.861	9.29414	365.155	4.38944	862.941	2.84837	700.832	3.12966	0.399682	-0.652256	-0.992413	1.01579	-0.037667	0.364562	0.387817	0.015122	-0.0226184	1.14169	0.78207	231.585	3.62143	327.818	1.27325	147.412	1.50389	360.654	7.91936	361.72	5.00587	492.748	4.77706];
% design_var = [0.3 0.5 0.5 0  0.6 -1.8095	0.942961	0.624199 0 0.4 0.5 0 0 0.1 0.1 300 3 300 3 300 3 300 3 300 3 300 3   0.6 -1.8095	0.942961	0.624199 0 0.4 0.5 0 0 0.1 0.1 300 3 300 3 300 3 300 3 300 3 300 3   0.6 -0.85244	-0.947325	1.27901 0 0.4 0.5 0 0 0.1 0.1 300 3 300 3 300 3 300 3 300 3 300 3   0.6 -0.85244	-0.947325	1.27901 0 0.4 0.5 0 0 0.1 0.1 300 3 300 3 300 3 300 3 300 3 300 3];
% design_var = [0.257762	0.5	0.684968	0.144497	0.8	-1.82451	0.942961	0.618965	0.00748531	0.306731	0.496725	0	0	-0.230119	0.0263507	306.167	2.85871	392.652	4.2889	313.274	2.80717	100	3	444.445	5.12517	324.32	3.22993	0.792829	-1.83832	0.942961	0.610959	3.41632e-06	0.95135	0.43961	0.00136405	0	0.0661623	0.753817	273.341	2.34034	300	1.69304	313.867	3.79415	142.127	2.92022	316.11	3.22161	343.532	5.22523	0.8	-0.862809	-0.94727	1.27901	0.00453765	0.405031	0.465053	0	-0.0018795	0.146399	0.119983	300	2.98751	353.405	1.93867	246.664	2.15187	132.213	4.20135	323.654	3.11215	355.93	3.19998	0.793659	-0.835258	-1.00093	1.26364	-0.000295183	0.467767	0.5	0.000212901	-0.000616317	0.568516	0.160292	272.877	2.4263	404.371	2.04128	289.187	3.11151	124.021	2.30454	373.639	3.16077	300	3];
% design_var = [0.5	0.5	0.75 0.25	0.5	-1.7095	1.142961	0.324199	0	0.4	0.1	0	0	0.1	0.1	300	3	300	3	300	3	300	3	300	3	300	3	0.5	-1.7095	1.142961	0.324199	0	0.4	0.1	0	0	0.1	0.1	300	3	300	3	300	3	300	3	300	3	300	3	0.5	-0.65244	-0.997325	1.27901	0	0.4	0.1	0	0	0.1	0.1	300	3	300	3	300	3	300	3	300	3	300	3	0.5	-0.65244	-0.997325	1.27901	0	0.4	0.1	0	0	0.1	0.1	300	3	300	3	300	3	300	3	300	3	300	3];
% [OF, cons] = objective_function_main(design_var);
%  
% 
% % Running the optimization
% % 
options = nsgaopt();                    % create default options structure
options.popsize = 100;                    % populaion size
options.maxGen  = 1000;                    % max generation

options.numObj = 1;                     % number of objectives
options.numVar = 96;                     % number of design variables
options.numCons = 9;                     % number of constraints
options.lb = design_var_lower';                   % lower bound of x
options.ub = design_var_upper';                   % upper bound of x
options.objfun = @objective_function_main;     % objective function handle
options.plotInterval = 1;               % interval between two calls of "plotnsga". 
options.useParallel = 'yes';
% options.poolsize = 2;
options.crossoverFraction = 0.3;
options.mutationFraction = 0.1;
options.initfun = {@initpop, 'initial_population1.txt'};
result = nsga2(options);                % begin the optimization!
% % % % 
% % % % % % copyfile('populations.txt', '20140422MO_ro_pop.txt');