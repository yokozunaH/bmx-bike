%% control_vector_field.m
%
% Description:
%   Wrapper function for autogen_control_vector_field.m
%   Computes the control vector field for the nonlinear state-space
%   dynamics of the cart-pendulum robot.
%
% Inputs:
%   x: the state vector, x = [x_cart; theta_pend; dx_cart; dtheta_pend];
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   g_ss: a 4x1 vector that maps control input u to dx (time derivative of
%       the state x).

function g_ss = control_vector_field(x,params)


theta_bw = x(4);
theta_fw = x(5);

%function g_ss = autogen_control_vector_field(m_bw,m_com,m_fw,theta_bw,theta_fw)
g_ss = autogen_control_vector_field(params.model.dyn.bw.m,...
                                    params.model.dyn.com.m,...
                                    params.model.dyn.fw.m,...
                                    theta_bw,...
                                    theta_fw);

end