P.gravity = 9.8;
% Aircraft Characteristics
aerosonde
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%% Compute trim conditions using 'mavsim_trim.slx'
% Initial airspeed
P.Va0 = 17;
gamma = 0/180*pi; % Desired flight path angle (radians)
R     = inf; % Desired radius (m) - use (+) for right handed orbit, 
P.Ts = 0.01; % Autopilot sample rate
%%
% Initial conditions
P.pn0    = -800;  % Initial North position (m)
P.pe0    = 0;  % Initial East position (m)
P.pd0    = 0;  % Initial Down position (negative altitude) (m)
P.u0     = eps; % Initial velocity along body x-axis (m/s)
P.v0     = 0;  % Initial velocity along body y-axis (m/s)
P.w0     = 0;  % Initial velocity along body z-axis (m/s)
P.phi0   = 0;  % Initial roll angle (rad)
P.theta0 = 0;  % Initial pitch angle (rad)
P.psi0   = 0;  % Initial yaw angle (rad)
P.p0     = 0;  % Initial body frame roll rate (rad/s)
P.q0     = 0;  % Initial body frame pitch rate (rad/s)
P.r0     = 0;  % Initial body frame yaw rate (rad/s)

% Wind conditions
P.w_mag = 0; % Wind speed at 6 m altitude (m/s)
P.w_dir = 0; % Wind direction at 6 m altitude (deg clockwise from north)

%%
% Run trim commands
[x_trim, u_trim] = compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% Reset initial conditions to trim conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

%% Uncomment following code for parts 5.6 and 5.7
%% Compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

%% Linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);