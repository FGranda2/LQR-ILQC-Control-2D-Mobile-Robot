% run_all: Script for automating comparisons between the LQR and the
%          ILQR controller.
%
% Usage: Change simulation setup in task_design(). The commands below run
%        the LQR and ILQC controller scripts, and plot the responses for
%        compraison. By default, intermediate .mat data files are saved to
%        and read from './results/'.
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


main_p1_lqr; % run lqr
main_p2_ilqc; % run ilqc
plot_comparison; % plot results