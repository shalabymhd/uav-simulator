function drawVehicle(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);       % inertial Down position
    u        = uu(4);       % body frame velocities
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);      % roll rate
    q        = uu(11);      % pitch rate     
    r        = uu(12);      % yaw rate    
    t        = uu(13);      % time

    % define persistent variables 
    persistent vehicle_pts;
    persistent vehicle_handle;
    
    % first time function is called, initialize plot and persistent vars
    if t<=0.1,
        figure(1), clf
        vehicle_pts = defineVehiclePoints;
        vehicle_handle = drawVehicleBody(vehicle_pts,pn,pe,pd,phi,theta,psi, [], 'normal');
        title('Spacecraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the view angle for figure
        axis([-10,10,-10,10,-10,10]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(vehicle_pts,pn,pe,pd,phi,theta,psi, vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicleBody
% return handle if 7th argument is empty, otherwise use 7th arg as handle
%=======================================================================
%
function handle = drawVehicleBody(NED,pn,pe,pd,phi,theta,psi, handle, mode)
  
  NED = rotate(NED,phi,theta,psi); % rotate spacecraft by phi, theta, psi
  NED = translate(NED,pn,pe,pd); % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  
  % plot spacecraft
  if isempty(handle),
    handle = plot3(NED(2,:),NED(1,:),-NED(3,:),'EraseMode', mode);
  else
    set(handle,'XData',NED(2,:),'YData',NED(1,:),'ZData',-NED(3,:));
    drawnow
  end
end
 

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi);

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translateXYZ

%=======================================================================
% defineVehiclePoints
%=======================================================================
function pts = defineVehiclePoints

scale      = 1/5;

fuse_l1    = 7*scale;
fuse_l2    = 4*scale;
fuse_l3    = 15*scale;
fuse_h     = 2*scale; %assumed to have a square tip base
fuse_w     = 2*scale;
wing_l     = 6*scale;
wing_w     = 20*scale;
tailwing_l = 3*scale;
tailwing_w = 10*scale;
tail_h     = 3*scale;

%Fuselage: 1,2,3,1,5,2,1,4,3,1,4,5,2,3,4,6,3,4,5,6,2,5,6...
%Front Wing: ...newpoint1,8,7,10,9,newpoint1,6...
%Tail Wing Horizontal: ...13,14,11,12,6...
%Tail Wing Vertical: 15,16,6

pts = [...
    fuse_l1, 0, 0;...   % pt 1
    fuse_l2, fuse_w/2, -fuse_h/2;... % pt 2
    fuse_l2, -fuse_w/2, -fuse_h/2;...   % pt 3
    fuse_l1, 0, 0;...  % pt 1
    fuse_l2, fuse_w/2, fuse_h/2;...   % pt 5
    fuse_l2, fuse_w/2, -fuse_h/2;...  % pt 2
    fuse_l1, 0, 0;...   % pt 1
    fuse_l2, -fuse_w/2, fuse_h/2;...   % pt 4
    fuse_l2, -fuse_w/2, -fuse_h/2;...   % pt 3
    fuse_l1, 0, 0;... % pt 1
    fuse_l2, -fuse_w/2, fuse_h/2;...   % pt 4
    fuse_l2, fuse_w/2, fuse_h/2;...  % pt 5
    fuse_l2, fuse_w/2, -fuse_h/2;...   % pt 2
    fuse_l2, -fuse_w/2, -fuse_h/2;...  % pt 3
    fuse_l2, -fuse_w/2, fuse_h/2;...   % pt 4
    -fuse_l3, 0, 0;...   % pt 6
    fuse_l2, -fuse_w/2, -fuse_h/2;...   % pt 3
    fuse_l2, -fuse_w/2, fuse_h/2;... % pt 4
    fuse_l2, fuse_w/2, fuse_h/2;...   % pt 5
    -fuse_l3, 0, 0;...  % pt 6
    fuse_l2, fuse_w/2, -fuse_h/2;...   % pt 2
    fuse_l2, fuse_w/2, fuse_h/2;...  % pt 5
    -fuse_l3, 0, 0;...   % pt 6
    -wing_l, 0, 0;... %newpoint 1
    -wing_l, wing_w/2, 0;...   % pt 8
    0, wing_w/2, 0;... % pt 7
    0, -wing_w/2, 0;...   % pt 10
    -wing_l, -wing_w/2, 0;...  % pt 9
    -wing_l, 0, 0;... %newpoint 1
    -fuse_l3, 0, 0;...   % pt 6
    -fuse_l3, -tailwing_w/2, 0;...   % pt 13
    -fuse_l3+tailwing_l, -tailwing_w/2, 0;...   % pt 14
    -fuse_l3+tailwing_l, tailwing_w/2, 0;... % pt 11
    -fuse_l3, tailwing_w/2, 0;...   % pt 12
    -fuse_l3, 0, 0;...  % pt 6
    -fuse_l3+tailwing_l, 0, 0;...   % pt 15
    -fuse_l3, 0, -tail_h;...  % pt 16
    -fuse_l3, 0, 0;...   % pt 6
    ]';
end
  