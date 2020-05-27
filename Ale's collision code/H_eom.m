%% H_eom.m
%
% Description:
%   Wrapper function for autogen_H.m
%   Computes H(q,q_dot) for the jumping robot.
%
% Inputs:
%   x: the state vector, x = [q; q_dot];
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   H: a 5x1 set of terms.

function H = H_eom(x,params)

theta_com = x(3);

dtheta_com = x(8);

g = params.model.dyn.g;
m_bw = params.model.dyn.bw.m;
m_com = params.model.dyn.com.m;
m_fw = params.model.dyn.fw.m;
bw_com_distance = params.model.geom.bw_com.l;
bw_com_init_angle = params.model.geom.bw_com.theta;
bw_fw_distance = params.model.geom.bw_fw.l;

%function H = autogen_H_eom(bw_com_distance,bw_com_init_angle,dtheta_bw,dtheta_fw,dtheta_com,dx_com,dy_com,fw_com_distance,fw_com_init_angle,g,m_bw,m_com,m_fw,theta_bw,theta_fw,theta_com)
H = autogen_H_eom(bw_fw_distance, bw_com_distance, bw_com_init_angle, dtheta_com, g, m_bw, m_com, m_fw, theta_com);

end

