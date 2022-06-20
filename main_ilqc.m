% main_p1_ilqc: Main script for Problem 2.1 ILQC controller design.
%
% --
% Control for Robotics
% AER1517 Spring 2022
% Assignment 2
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistant: 
% SiQi Zhou
% siqi.zhou@robotics.utias.utoronto.ca
% Lukas Brunke
% lukas.brunke@robotics.utias.utoronto.ca
% Adam Hall
% adam.hall@robotics.utias.utoronto.ca
%
% --
% Revision history
% [20.01.31, SZ]    first version
% Modified and completed by Francisco Granda
clear all;
close all;

%% General
% add subdirectories
addpath(genpath(pwd));

% add task
task_ilqc = task_design();
N = length(task_ilqc.start_time:task_ilqc.dt:task_ilqc.end_time);

% add model
const_vel = 1; % assume constant forward speed
model = generate_model(const_vel);

% save directory
save_dir = './results/';

% initialize controller
load(strcat(save_dir, 'lqr_controller'));
controller_ilqc = controller_lqr;

% flags
plot_on = true;
save_on = true;

%% Iterative Linear Quadratic Controller
% =========================== ILQC Design ==========================
% Initialization Parameters
dt = task_ilqc.dt;
v_bar = const_vel;
Q_t = task_ilqc.cost.params.Q_t;
Q_s = task_ilqc.cost.params.Q_s;
R_s = task_ilqc.cost.params.R_s;
x_goal = task_ilqc.goal_x;

% Main Loop
for j = 1:task_ilqc.max_iteration
    % Forward Pass
    if j == 1
        u_i = controller_ilqc;
    end
    x_i = zeros(2,size(controller_lqr,2));
    for i = 0:size(controller_lqr,2)
        if i == 0
            x1_0 = task_ilqc.start_x(1);
            x2_0 = task_ilqc.start_x(2);
            u_0 = u_i(:,1);
            x_knext = [x1_0;x2_0]+dt*...
                [v_bar*sin(x2_0);u_0(1)+x1_0*u_0(2)+x2_0*u_0(3)];
            x_i(1,i+1) = x_knext(1);
            x_i(2,i+1) = x_knext(2);
        else
            x1_0 = x_i(1,i);
            x2_0 = x_i(2,i);
            x_knext = [x1_0;x2_0]+dt*...
                [v_bar*sin(x2_0);u_i(1,i)+x1_0*u_i(2,i)+x2_0*u_i(3,i)];
            x_i(1,i+1) = x_knext(1);
            x_i(2,i+1) = x_knext(2);
        end
    end
    
    % Backward Pass
    for i = size(controller_lqr,2)+1:-1:1
       
        if i == size(controller_lqr,2)+1
            [s_N(i),s_NN{i},S_N{i}] = ...
                terminal_cost_quad(Q_t,x_goal,[x_i(1,i);x_i(2,i)]);
        else
            % Dynamic Matrices
            u_k = u_i(1,i)+x_i(1,i)*u_i(2,i)+x_i(2,i)*u_i(3,i);
            x_bar = [x_i(1,i);x_i(2,i)];
            [A_k,B_k] = mobile_robot_lin(x_bar,dt,v_bar);
            % Stage Cost
            [q_k,q_kk,Q_k,r_k,R_k,P_k] = ...
                stage_cost_quad(Q_s,R_s,x_goal,dt,x_bar,u_k);
            % Policy Update
            s_N_in = s_N(i+1);
            s_NN_in = s_NN{i+1};
            S_N_in = S_N{i+1};
            [u_i(1,i),u_i(2:3,i),s_N(i),s_NN{i},S_N{i}] = ...
                update_policy(A_k,B_k,q_k,q_kk,Q_k,r_k,...
                R_k,P_k,s_N_in,s_NN_in,S_N_in,x_bar,u_k);
            
        end
    end
end
controller_ilqc = u_i;
%% Simulation
sim_out_ilqc = mobile_robot_sim(model, task_ilqc, controller_ilqc);
%sim_out_ilqc = mobile_robot_sim_test(model, task_ilqc, controller_ilqc,0.5);
fprintf('trajectory cost: %.2f\n', sim_out_ilqc.cost);
fprintf('\n\ntarget state [%.3f; %.3f]\n', task_ilqc.goal_x);
fprintf('reached state [%.3f; %.3f]\n', sim_out_ilqc.x(:,end));

%% Plots
if plot_on
    plot_results(sim_out_ilqc);
end

%% Save controller and simulation results
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'ilqc_controller'), 'controller_ilqc', ...
        'sim_out_ilqc', 'task_ilqc'); 
end