% main_p1_lqr: Main script for Problem 2.1 LQR controller design.
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

% define task
task_lqr = task_design();
N = length(task_lqr.start_time:task_lqr.dt:task_lqr.end_time);

% add model
const_vel = 1; % desired forward speed
model = generate_model(const_vel);

% initialize controller
controller_lqr = zeros(3, N-1);

% save directory
save_dir = './results/';

% flags
plot_on = true;
save_on = true;

%%LQR Controller
% =========================== [TODO] LQR Design ===========================
A_lin = [0,1;0,0];
B_lin = [0;1];
Q = task_lqr.cost.params.Q_s;
R = task_lqr.cost.params.R_s;
K = -lqr(A_lin,B_lin,Q,R);
K_new = [-K*task_lqr.goal_x;K'];
controller_lqr = repmat(K_new,1,size(controller_lqr,2));
%% Simulation
sim_out_lqr = mobile_robot_sim(model, task_lqr, controller_lqr);
%sim_out_lqr = mobile_robot_sim_test(model, task_lqr, controller_lqr,0.5);
fprintf('--- LQR ---\n\n');
fprintf('trajectory cost: %.2f\n', sim_out_lqr.cost);
fprintf('target state [%.3f; %.3f]\n', task_lqr.goal_x);
fprintf('reached state [%.3f; %.3f]\n', sim_out_lqr.x(:,end));

%% Plots
if plot_on
    plot_results(sim_out_lqr);
end

%% Save controller and simulation results
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'lqr_controller'), 'controller_lqr', ...
        'sim_out_lqr', 'task_lqr'); 
end