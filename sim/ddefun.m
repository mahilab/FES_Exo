function X_dot = ddefun(t,X,Z)
    
    FES_factor = 1.0;

    I = .2084 + 0.0693;
    C = 0.1215;
    
    Kp = 100;
    Kd = 1.25*8;
    
    Kp_lag = Kp*FES_factor;
    Kd_lag = 1.25*FES_factor*0;

    xlags = Z(:,1);
    x_lag = xlags(1);
    xd_lag = xlags(2);

    x_des = sin(2*t);
    Tau = Kp_lag * (x_des - x_lag) + Kd_lag * (-xd_lag) + Kp * (x_des - X(1)) + Kd * (-X(2));

    x_dd = 1/I * (Tau - C * X(2));

    X_dot = [ X(2); 
             x_dd];
end