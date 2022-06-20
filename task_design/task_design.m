% task_design: Function for specifying the task to fulfilled by the mobile
%              robot as well as the parameters for simulation.
%
% Output:
%       task:	A structure that describes the task to be
%               fulfilled/simulated. This includes parameters of the cost
%               function, initial and final time, sampling interval,
%               initial and goal states.
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

function task = task_design( )
    % defines a task for a controller to execute
    task = struct;

    % intial and final simulation time, and discrete time step
    task.start_time = 0;	% start time
    task.end_time = 15;     % end time
    task.dt = 0.02;         % discrete time step

    % start and final states
    task.start_x = [0; 0];   % [y; h]
    %task.start_x = [0; pi];   % [y; h]
    task.goal_x = [-5; 0];   % [y; h]
    
    % stage cost function parameters
    task.cost.params.Q_s = diag([1, 1]);
    task.cost.params.R_s = 20;
    
    % terminal cost function parameters
	task.cost.params.Q_t = diag([1, 1]);
    
    % maximum ILQC iterations
    task.max_iteration  = 15;
end