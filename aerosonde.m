%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
% Physical parameters of airframe
P.mass = 13.5; % kg
P.Jx   = 0.8244; % kg-m2
P.Jy   = 1.135; % kg-m2
P.Jz   = 1.759; % kg-m2
P.Jxz  = 0.1204; % kg-m2

% Aircraft aerodynamics
P.S_wing        = 0.55; % m2
P.b             = 2.8956; % m
P.c             = 0.18994; % m
P.S_prop        = 0.2027; % m2
P.rho           = 1.2682; % kg/m3
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712; % rad

% Aerodynamic coefficients
P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%