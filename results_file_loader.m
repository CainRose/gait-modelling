%% Loading torque values from files
global temporary_time t_sen_lg1_j1 t_sen_lg1_j2 t_sen_lg1_j3 t_sen_lg2_j1 t_sen_lg2_j2 t_sen_lg2_j3 
global t_sen_lg3_j1 t_sen_lg3_j2 t_sen_lg3_j3 t_sen_lg4_j1 t_sen_lg4_j2 t_sen_lg4_j3
global theta_dot_sen_lg1_j1 theta_dot_sen_lg1_j2 theta_dot_sen_lg1_j3 theta_dot_sen_lg2_j1 theta_dot_sen_lg2_j2 theta_dot_sen_lg2_j3
global theta_dot_sen_lg3_j1 theta_dot_sen_lg3_j2 theta_dot_sen_lg3_j3 theta_dot_sen_lg4_j1 theta_dot_sen_lg4_j2 theta_dot_sen_lg4_j3
global theta_sen_lg1_j1 theta_sen_lg1_j2 theta_sen_lg1_j3 theta_sen_lg2_j1 theta_sen_lg2_j2 theta_sen_lg2_j3
global theta_sen_lg3_j1 theta_sen_lg3_j2 theta_sen_lg3_j3 theta_sen_lg4_j1 theta_sen_lg4_j2 theta_sen_lg4_j3
global body_pitch body_height_calc
global leg1_ee_velocity leg2_ee_velocity leg3_ee_velocity leg4_ee_velocity
global leg1_ee_position leg2_ee_position leg3_ee_position leg4_ee_position
temporary_load = load('t_sen_lg1.mat');
temporary_time = temporary_load.ans.time;
temporary_data = temporary_load.ans.data;
t_sen_lg1_j1 = temporary_data(:,1);
t_sen_lg1_j2 = temporary_data(:,2);
t_sen_lg1_j3 = temporary_data(:,3);

temporary_load = load('t_sen_lg2.mat');
temporary_data = temporary_load.ans.data;
t_sen_lg2_j1 = temporary_data(:,1);
t_sen_lg2_j2 = temporary_data(:,2);
t_sen_lg2_j3 = temporary_data(:,3);

temporary_load = load('t_sen_lg3.mat');
temporary_data = temporary_load.ans.data;
t_sen_lg3_j1 = temporary_data(:,1);
t_sen_lg3_j2 = temporary_data(:,2);
t_sen_lg3_j3 = temporary_data(:,3);

temporary_load = load('t_sen_lg4.mat');
temporary_data = temporary_load.ans.data;
t_sen_lg4_j1 = temporary_data(:,1);
t_sen_lg4_j2 = temporary_data(:,2);
t_sen_lg4_j3 = temporary_data(:,3);

%% Loading position values from files
temporary_load = load('theta_sen_lg1.mat');
temporary_data = temporary_load.ans.data;
theta_sen_lg1_j1 = temporary_data(:,1);
theta_sen_lg1_j2 = temporary_data(:,2);
theta_sen_lg1_j3 = temporary_data(:,3);

temporary_load = load('theta_sen_lg2.mat');
temporary_data = temporary_load.ans.data;
theta_sen_lg2_j1 = temporary_data(:,1);
theta_sen_lg2_j2 = temporary_data(:,2);
theta_sen_lg2_j3 = temporary_data(:,3);

temporary_load = load('theta_sen_lg3.mat');
temporary_data = temporary_load.ans.data;
theta_sen_lg3_j1 = temporary_data(:,1);
theta_sen_lg3_j2 = temporary_data(:,2);
theta_sen_lg3_j3 = temporary_data(:,3);

temporary_load = load('theta_sen_lg4.mat');
temporary_data = temporary_load.ans.data;
theta_sen_lg4_j1 = temporary_data(:,1);
theta_sen_lg4_j2 = temporary_data(:,2);
theta_sen_lg4_j3 = temporary_data(:,3);

%% Loading velocity values from files
temporary_load = load('theta_dot_sen_lg1.mat');
temporary_data = temporary_load.ans.data;
theta_dot_sen_lg1_j1 = temporary_data(:,1);
theta_dot_sen_lg1_j2 = temporary_data(:,2);
theta_dot_sen_lg1_j3 = temporary_data(:,3);

temporary_load = load('theta_dot_sen_lg2.mat');
temporary_data = temporary_load.ans.data;
theta_dot_sen_lg2_j1 = temporary_data(:,1);
theta_dot_sen_lg2_j2 = temporary_data(:,2);
theta_dot_sen_lg2_j3 = temporary_data(:,3);

temporary_load = load('theta_dot_sen_lg3.mat');
temporary_data = temporary_load.ans.data;
theta_dot_sen_lg3_j1 = temporary_data(:,1);
theta_dot_sen_lg3_j2 = temporary_data(:,2);
theta_dot_sen_lg3_j3 = temporary_data(:,3);

temporary_load = load('theta_dot_sen_lg4.mat');
temporary_data = temporary_load.ans.data;
theta_dot_sen_lg4_j1 = temporary_data(:,1);
theta_dot_sen_lg4_j2 = temporary_data(:,2);
theta_dot_sen_lg4_j3 = temporary_data(:,3);

temporary_body = load('body_movement.mat');
tempo_body = temporary_body.ans.data;
body_pitch = tempo_body(:,1);
body_height_calc = tempo_body(:,2);

temporary_load = load('leg1_ee.mat');
tempo_data = temporary_load.ans.data;
leg1_ee_position = tempo_data(:,2);
leg1_ee_velocity = sqrt(((tempo_data(:,3)).^2));

temporary_load = load('leg2_ee.mat');
tempo_data = temporary_load.ans.data;
leg2_ee_position = tempo_data(:,2);
leg2_ee_velocity = sqrt(((tempo_data(:,3)).^2));

temporary_load = load('leg3_ee.mat');
tempo_data = temporary_load.ans.data;
leg3_ee_position = tempo_data(:,2);
leg3_ee_velocity = sqrt(((tempo_data(:,3)).^2));

temporary_load = load('leg4_ee.mat');
tempo_data = temporary_load.ans.data;
leg4_ee_position = tempo_data(:,2);
leg4_ee_velocity = sqrt(((tempo_data(:,3)).^2));