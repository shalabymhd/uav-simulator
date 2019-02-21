% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%
function out = forces_moments(x, delta, wind, P)
    % Relabel the inputs
    % States
    pn  = x(1); pe    = x(2); pd  = x(3);
    u   = x(4); v     = x(5); w   = x(6);
    phi = x(7); theta = x(8); psi = x(9);
    p   = x(10); q    = x(11);r   = x(12);
    
    % Control inputs (in radians for control surfaces and 0.0-1.0 for
    % throttle)
    delta_e = delta(1); delta_a = delta(2);
    delta_r = delta(3); delta_t = delta(4);
    
    % Wind data
    u_w = wind(1); % Wind along body x-axis (m/s)
    v_w = wind(2); % Wind along body y-axis (m/s)
    w_w = wind(3); % Wind along body z-axis (m/s)
    
    % Transform wind data from body-frame to inertial-frame
    w_n = cos(theta)*cos(psi)*u_w + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v_w + (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w_w; % North wind (m/s)
    w_e = cos(theta)*sin(psi)*u_w + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v_w + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w_w; % East wind (m/s)
    w_d = -sin(theta)*u_w + sin(phi)*cos(theta)*v_w + cos(phi)*cos(theta)*w_w; % Down wind (m/s)
    
    % Compute body-frame airspeed vector
    u_r = u - u_w; % Relative wind along body x-axis (m/s)
    v_r = v - v_w; % Relative wind along body y-axis (m/s)
    w_r = w - w_w; % Relative wind along body z-axis (m/s)
    
    % Compute air data
    Va = sqrt(u_r^2 + v_r^2 + w_r^2); % Airspeed (m/s)
    alpha = atan(w_r/u_r); % Angle of Attack (rad)
    beta = asin(v_r/(sqrt(u_r^2 + v_r^2 + w_r^2))); % Sideslip Angle (rad)
    qS = 0.5*P.rho*Va^2*P.S_wing; % Dynamic pressure X Wing surface area
    
    % Compute aerodynamic coefficients of lift and drag
    sigma = (1+exp(-P.M*(alpha-P.alpha0))+exp(P.M*(alpha+P.alpha0)))/((1+exp(-P.M*(alpha-P.alpha0)))*(1+exp(P.M*(alpha+P.alpha0))));clc % Sigmoid blending function
    C_L = (1-sigma)*(P.C_L_0 + P.C_L_alpha*alpha) + sigma*(2*sign(alpha)*(sin(alpha))^2*cos(alpha)); % Coefficient of lift
    C_D = P.C_D_p + (P.C_L_0+P.C_L_alpha*alpha)^2/(pi*P.e*((P.b)^2/P.S_wing)); % Coefficient of drag

    % Compute force coefficients acting on body-frame
    C_X = -C_D*cos(alpha) + C_L*sin(alpha); % Force coefficient due to lift and drag along body x-axis
    C_X_q = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha); % Force coefficient due to pitch rate along body x-axis
    C_X_delta_e = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha); % Force coefficient due to del_e along body x-axis
    C_Z = -C_D*sin(alpha) - C_L*cos(alpha); % Force coefficient due to lift and drag along body z-axis
    C_Z_q = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha); % Force coefficient due to pitch rate along body z-axis
    C_Z_delta_e = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha); % Force coefficient due to del_e along body z-axis
    
    % Compute external forces and torques on aircraft
    Force(1) =  -P.mass*P.gravity*sin(theta) + ...
                qS * (C_X + C_X_q*P.c/(2*Va)*q + C_X_delta_e*delta_e) + ...
                0.5*P.rho*P.S_prop*P.C_prop * ((P.k_motor*delta_t)^2 - Va^2); % Forces along the x-axis
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi) + ...
                qS * (P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b/(2*Va)*p + P.C_Y_r*P.b/(2*Va)*r + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r); % Forces along the y-axis
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi) + ...
                qS * (C_Z + C_Z_q*P.c/(2*Va)*q + C_Z_delta_e*delta_e); % Forces along the z-axis

    Torque(1) = qS * (P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*P.b/(2*Va)*p + P.C_ell_r*P.b/(2*Va)*r + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r)) - P.k_T_P*(P.k_Omega*delta_t)^2; % Moments about the x-axis
    Torque(2) = qS * (P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c/(2*Va)*q + P.C_m_delta_e*delta_e)); % Moments about the y-axis
    Torque(3) = qS * (P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b/(2*Va)*p + P.C_n_r*P.b/(2*Va)*r + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r)); % Moments about the z-axis
    
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



