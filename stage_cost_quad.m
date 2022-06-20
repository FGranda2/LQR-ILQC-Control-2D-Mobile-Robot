function [q_k,q_kk,Q_k,r_k,R_k,P_k] = ...
    stage_cost_quad(Q_s,R_s,x_goal,delta_t,x_bar,u_bar)
% Compute stage cost
q_k = delta_t*(R_s*u_bar^2/2 + Q_s(1,1)*(x_bar(1)-x_goal(1))^2/2+...
    Q_s(2,2)*(x_bar(2)-x_goal(2))^2/2);

q_kk = [Q_s(1,1)*delta_t*(x_bar(1)-x_goal(1)),...
    Q_s(2,2)*delta_t*(x_bar(2)-x_goal(2))];
q_kk = q_kk.';

Q_k = delta_t*diag([Q_s(1,1),Q_s(2,2)]);

r_k = R_s*delta_t*u_bar;

R_k = R_s*delta_t;

P_k = [0,0];
end

