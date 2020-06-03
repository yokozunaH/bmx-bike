function H = autogen_H_eom(g,l_com,m_bike,m_mw,theta_bike)
%AUTOGEN_H_EOM
%    H = AUTOGEN_H_EOM(G,L_COM,M_BIKE,M_MW,THETA_BIKE)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    30-May-2020 13:20:14

H = [g.*l_com.*cos(theta_bike).*(m_bike+m_mw);0.0];
