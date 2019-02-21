function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R)
% Va is the desired airspeed (m/s)
% gamma is the desired flight path angle (radians)
% R is the desired radius (m) - use (+) for right handed orbit, 
%                                   (-) for left handed orbit
%% Initial Guesses for trim conditions
    % x0 defines [pn pe pd u v w phi theta psi p q r]'
    x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
    % u0 defines [delta_e delta_a delta_r delta_t]'
    u0 = [0; 0; 0; 1];
    % y0 = [Airspeed flight_path_angle AOA]'
    y0 = [Va; gamma; 0];
    % Constraints on states, inputs, and outputs
    ix = []; iu = [] ; iy = [1,2,3]; %*******in the book this was [1,3]
    idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];
    % Change in states and output
    h_dot_star = Va*sin(gamma);
    psi_dot_star = Va/R;%*cos(gamma);
    dx0 = [0; 0; -h_dot_star; 0; 0; 0; 0; 0; psi_dot_star; 0; 0; 0];
%% Compute trim conditions
    % tilde replaces y_trim since it is required
    [x_trim,u_trim,~,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);

%% Check to make sure that the linearization worked (should be small)
    Norm = norm(dx_trim(3:end)-dx0(3:end))