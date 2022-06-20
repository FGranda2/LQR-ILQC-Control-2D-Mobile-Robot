function [theta_ff,theta_fb,s_N_out,s_NN_out,S_N_out] = ...
    update_policy(A_k,B_k,q_k,q_kk,Q_k,r_k,...
    R_k,P_k,s_N_in,s_NN_in,S_N_in,x_bar,u_k)
% Update policy parameters for backwards solving
% Functions
g_k = r_k + B_k.' * s_NN_in;
G_k = P_k + B_k.' * S_N_in*A_k;
H_k = R_k + B_k.' * S_N_in*B_k;
% Dinputs
du_ff = -inv(H_k)*g_k;
K_k = -inv(H_k)*G_k;
% S compute
s_N_out = q_k + s_N_in + 1/2*du_ff.'*H_k*du_ff + du_ff.'*g_k;
s_NN_out = q_kk + A_k.'*s_NN_in+K_k.'*H_k*du_ff+K_k.'*g_k+G_k.'*du_ff;
S_N_out = Q_k + A_k.'*S_N_in*A_k + K_k.'*H_k*K_k + K_k.'*G_k + G_k.'*K_k;
% Update Control Policy
theta_ff = du_ff + u_k - K_k(1)*x_bar(1)-K_k(2)*x_bar(2);
theta_fb(1) = K_k(1);
theta_fb(2) = K_k(2);
end

