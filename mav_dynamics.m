function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%==========================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%==========================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume 
% DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model
% sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%==========================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%==========================================================================
%
function sys=mdlDerivatives(t,x,uu,P)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
%% Scalar formulation of kinematics and dynamics equations

%     gamma  = P.Jx*P.Jz - P.Jxz^2;
%     gamma1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/gamma;
%     gamma2 = (P.Jz*(P.Jz-P.Jy) + P.Jxz^2)/gamma;
%     gamma3 = P.Jz/gamma;
%     gamma4 = P.Jxz/gamma;
%     gamma5 = (P.Jz - P.Jx)/P.Jy;
%     gamma6 = P.Jxz/P.Jy;
%     gamma7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/gamma;
%     gamma8 = P.Jx/gamma;
% 
%     pndot = cos(theta)*cos(psi)*u + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v + (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
%     pedot = cos(theta)*sin(psi)*u + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v + (cos(phi)*sin(theta)*cos(psi)-sin(phi)*cos(psi))*w;
%     pddot = -sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;
%     udot = r*v - q*w + fx/P.mass;
%     vdot = p*w - r*u + fy/P.mass;
%     wdot = q*u - p*v + fz/P.mass;
%     phidot = 1*p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
%     thetadot = 0*p + cos(phi)*q - sin(phi)*r;
%     psidot = 0*p + sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;
%     pdot = gamma1*p*q - gamma2*q*r + gamma3*ell + gamma4*n;
%     qdot = gamma5*p*r - gamma6*(p^2-r^2) + m/P.Jy;
%     rdot = gamma7*p*q - gamma1*q*r + gamma4*ell + gamma8*n;
% 
% sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; ...
%     psidot; pdot; qdot; rdot];
%%
% As opposed to the scalar formulation presented in Beard's Small Unmanned
% Aerial Vehicles book, another way to define the kinematics and dynamics
% of the aircraft is through the matrix formulation of the 
% equations. To do so, comment the above section and uncomment the section
% below.

%% Matrix formulation of kinematics and dynamics equations

     % Build State Matrices
     Pos = [pn pe pd]'; Vb_g = [u v w]'; wb_i = [p q r]';
     
     % Build forces and moments matrices
     Fb = [fx fy fz]'; mb = [ell m n]';
     
     % Build the aircraft symmetric inertia matrix
     J = [P.Jx 0 -P.Jxz; 0 P.Jy 0; -P.Jxz 0 P.Jz];
     
     % Build body frame to vehicle frame transformation matrix
     Rv_b = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
             cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
             -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
     
     % Build Euler angle time derivative transformation matrix (Eq. 3.3)
     Re_v = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);...
             0 cos(phi) -sin(phi);...
             0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
     
     % Define kinematics and dynamics
     Posdot = Rv_b*Vb_g;% Posdot is the column matrix of position time derivatives
     Vb_gdot = Fb/P.mass - cross(wb_i,Vb_g);%Time derivative about the inertial frame
     Psidot = Re_v*wb_i;% Psidot is the column matrix of Euler angle time derivatives
     wb_idot = inv(J)*(cross(-wb_i,J*wb_i)+mb);%Time derivative about the inertial frame
 sys = [Posdot' Vb_gdot' Psidot' wb_idot'];

% end mdlDerivatives

%
%==========================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%==========================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%==========================================================================
% mdlOutputs
% Return the block outputs.
%==========================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%==========================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%==========================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%==========================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%==========================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
