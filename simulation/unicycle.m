% unicycle: Continuous-time model of the mobile robot.
%
% Inputs:
%       state: A column vector containing the current state of the system
%              [y; h].
%       input: A scalar specifing current turning rate input omega (rad/s).
%       model: A structure containing model parameters, dynamics model
%              handles, and the dimensions of the input and the state.
%
% Output:
%       states_dot: A vector containing the instantaneous rate of change of
%           the state [y_dot; h_dot]
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

function [ states_dot ] = unicycle(state, input, model)
    % extract inputs
    omega = input; % turning rate
    v = model.param.const_vel; % desired forward speed
    
    % compute states_dot = f(states, inputs)
    states_dot(1,1) = v*sin(state(2)); % y_dot
    states_dot(2,1) = omega; % h_dot
end