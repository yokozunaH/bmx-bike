function C_all = autogen_constraints(bw_fw_distance,r_bw,r_fw,theta_bw,theta_fw,theta_com,x_bf,y_bf)
%AUTOGEN_CONSTRAINTS
%    C_ALL = AUTOGEN_CONSTRAINTS(BW_FW_DISTANCE,R_BW,R_FW,THETA_BW,THETA_FW,THETA_COM,X_BF,Y_BF)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    08-May-2020 17:06:40

t2 = r_bw.*theta_bw;
t3 = -r_bw;
C_all = [t2+x_bf;t3+y_bf;t3+y_bf+bw_fw_distance.*sin(theta_com);t2-r_fw.*theta_fw];
