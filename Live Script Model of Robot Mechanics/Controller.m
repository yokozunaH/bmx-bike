function [tau_d,e_total,e_prev,status] = Controller(state,eint,prevError,status)

eintmax = 3000;

if (eint>eintmax)
    eint = eintmax;
elseif (eint<-eintmax)
    eint = -eintmax;
end

params = init_params;
switch params.sim.trick
    case 'Backflip'
        Kp = 1;
        Ki = 0;
        Kd = 0; %1;

        set = -126; % speed calculated from winter quarter 
        error = set-state;
        tau_d = Kp * error + Ki * eint + Kd *(error-prevError)/params.sim.dt;
        e_total = eint+error;
        e_prev = error;
        
        if status ==  'On Ramp'
            tau_d = 0; 
        end
    
    case 'Wheelie'
        Kp = 9; %150; 
        Ki = 0.5; %0.05; 
        Kd = 1.8; %200; 
    
        set = pi/2 - params.model.geom.bw_com.theta; 

        error = set-state;
        %display(error)

        e_total = eint+error;
        e_prev = error;

        tau_d = Kp * error + Ki * eint + Kd *(error-prevError)/params.sim.dt;

        if error < 0
            %display("neg")
            status = "neg"; 
        else
            status = "pos";
        end

end



end
