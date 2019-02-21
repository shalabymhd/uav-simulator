% Simulation conditions

P.gravity = 9.81; % m/s2

% Initial conditions
P.pn0    = -800;  % Initial North position (m)
P.pe0    = 0;  % Initial East position (m)
P.pd0    = 0;  % Initial Down position (negative altitude) (m)
P.u0     = 40;%eps;  % Initial velocity along body x-axis (m/s)
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
