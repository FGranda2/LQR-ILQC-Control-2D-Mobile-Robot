% generate_model: Function for defining system dynamics and basic
%                 parameters.
%
% Input:
%       const_vel: A scalar specifing the desired forward speed of the
%                  mobile robot in (m/s)
%
% Output:
%       model: A structure containing model parameter, dynamics model
%              handles, and the dimensions of the input and the state.
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

function [ model ] = generate_model(const_vel)
    % predefined forward speed
    model.param.const_vel = const_vel;
    
    % model handles
    model.f = @unicycle; % for controller design
    model.f_test = @unicycle_test; % for testing
    
    % input and output dimensions
    model.input_dim = 2;
    model.state_dim = 3;
end