%IDNLGREY model file (discrete-time nonlinear model) : 
%xn = x(t+Ts) : state update values in discrete-time case (A column vector with Nx entries)
%y : outputs values (A column vector with Ny entries)
function [dx, y] = quadrotor_m(t, x, u, a,b,c,d, m, I_x, I_y, I_z, K_T, K_Q, varargin)
    
    dx = quadrotor_odefun(x, u, a,b,c,d, m, I_x, I_y, I_z, K_T, K_Q);       

    %% addtional effects here
    g = 9.80665;
    terminal_velocity = 15; 
    terminal_rotation_rate = 4 * deg2rad(360); 
    
    % air resistance
    air_resistance = -x(7:9) * g / terminal_velocity;       % *0.6538
    dx(7:9) = dx(7:9) + air_resistance;
    
    %rotational air resitance
    rot_air_resistance = -x(10:12) * deg2rad(400) / terminal_rotation_rate; % *0.2778
    dx(10:12) = dx(10:12) + rot_air_resistance;
    %
    global frame_height;
    if (on_ground(x(3), frame_height)) && (dx(9) <= 0)   % on ground 
        dx(9) = 0;
    end

    y = x(1:12);         % update outputs
    
end
