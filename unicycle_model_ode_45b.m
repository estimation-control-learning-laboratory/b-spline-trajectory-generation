function dotz = unicycle_model_ode_45b(t, z)

    % Proportional 
    P = 4;
    % Derivative
    D = 4;
    
    global spline spline_dot spline_dot_dot;
    
    


    x  = z(1);
    y =  z(2);
    v = z(3);
    theta = z(4) ;

    
    dot_x = v*cos(theta);
    dot_y = v*sin(theta);
    
    ref_position = fnval(spline, t);
    ref_velocity = fnval(spline_dot, t);
    ref_accel = fnval(spline_dot_dot, t);
    
    position = [x;y];
    velocity = [dot_x; dot_y];

    % The inputs are the acceleration, a, and omega. 
    inputs = (1/v) * [v*cos(theta), v*sin(theta); -sin(theta), cos(theta)] * ...
        (ref_accel + P*(ref_position - position) + D*(ref_velocity - velocity));
    
    
    dotz = [dot_x; dot_y; inputs];

end