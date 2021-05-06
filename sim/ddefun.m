function X_dot = ddefun(t,X,Z)

    global Kd;
    
    FES_factor = 1.0;

    I = .2084 + 0.0693;
    C = 0.1215;
    
    Kp = 100;
%     Kd = 1.25*3;
    
    Kp_fes = Kp*FES_factor;
    Kd_fes = 1.25*3.0;
    Ki_fes = 0;

    xlags = Z(:,1);
    x_lag = xlags(1);
    xd_lag = xlags(2);
    xi_lag = xlags(3);

    x_des = sin(2*t);
    Tau = Kp_fes * (x_des - x_lag) + Kd_fes * (-xd_lag) + Ki_fes * -xi_lag + Kp * (x_des - X(1)) + Kd * (-X(2));

    x_dd = 1/I * (Tau - C * X(2));

    X_dot = [ X(2); 
             x_dd
             (X(1)-x_des)];
end