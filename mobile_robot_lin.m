function [A_k,B_k] = mobile_robot_lin(x_bar,delta_t,v_bar)
% Compute dynamic system matrices
% Partial derivatives matrix
A_k = [1,delta_t*v_bar*cos(x_bar(2));...
       0,1];
B_k = [0;delta_t];
end

