function E = energy(x,params)

theta_bike = x(1);
dtheta_mw = x(4);
dtheta_bike = x(3);

g = params.model.dyn.g;
I_bike = params.model.dyn.com.I;
I_mw = params.model.dyn.mw.I;
l_com = params.model.dyn.com.l;
m_bike = params.model.dyn.com.m;
m_mw = params.model.dyn.mw.m;

E = autogen_energy(I_bike,I_mw,dtheta_mw,dtheta_bike,g,l_com,m_bike,m_mw,theta_bike)

end
