function main_path_following
    close all; clc;
    
    %% 1) Suppose we already have an optimal path from P1/P2
    %    For demonstration, let's just load or define some path.
    %    In practice, you'd run your main_assignment.m (or obstacles version),
    %    get x_path, y_path, and store them. Here, let's do a placeholder:

    load('myOptimalPath.mat','x_path','y_path'); 
    % or define a simple example if you don't have a file:
    % x_path = linspace(0,400,401);
    % y_path = 10*sin(2*pi*x_path/400);

    %% 2) Set up guidance parameters
    guidanceParams.lookAheadDist = 10;  % look-ahead distance [m]
    guidanceParams.Kphi          = 1.0; % bank-angle feedback gain
    guidanceParams.phiMax        = deg2rad(45); % saturate bank angle
    guidanceParams.x_path        = x_path;
    guidanceParams.y_path        = y_path;

    %% 3) Simulate closed-loop from t=0 to t=30 (or adjust)
    tspan = [0 30];
    % initial condition: x(0)=0, y(0)=0, chi(0)=90deg
    x0 = [0; 0; deg2rad(90)];
    
    [tSol, xSol] = ode45(@(t,x) aircraftDynamics(t,x,guidanceParams), tspan, x0);

    %% 4) Plot results
    figure('Name','Closed-Loop Path Following','Color','w');
    hold on; grid on; axis equal;
    plot(x_path, y_path, 'r--','LineWidth',1.5,'DisplayName','Ref Path');
    plot(xSol(:,1), xSol(:,2), 'b-','LineWidth',1.5,'DisplayName','Closed-Loop');
    legend('Location','best');
    xlabel('x [m]'); ylabel('y [m]');
    title('Path-Following: Comparison of Reference vs. Closed-Loop');

    figure('Name','Heading vs Time','Color','w');
    subplot(2,1,1);
    plot(tSol, rad2deg(xSol(:,3)), 'b-','LineWidth',1.5);
    grid on; ylabel('\chi [deg]'); xlabel('t [s]');
    title('Aircraft Heading');
    
    subplot(2,1,2);
    % For completeness, we might store the commanded bank angle in a global or pass out from guidance
    % But let's just re-compute quickly:
    phiCmd = zeros(size(tSol));
    for k=1:length(tSol)
       phiCmd(k) = guidanceLaw(xSol(k,:)', guidanceParams);
    end
    plot(tSol, rad2deg(phiCmd), 'r-','LineWidth',1.5);
    grid on; ylabel('\phi_{cmd} [deg]'); xlabel('t [s]');
    title('Commanded Bank Angle');
end

%% ===================================================================== %%
function dx = aircraftDynamics(~, x, guidanceParams)
    % x = [ xPos; yPos; chi ]
    xPos = x(1);
    yPos = x(2);
    chi  = x(3);

    % guidance law => phi command
    phiCmd = guidanceLaw(x, guidanceParams);

    % saturate bank angle
    phiCmd = max(-guidanceParams.phiMax, min(guidanceParams.phiMax, phiCmd));

    % system parameters
    V = 20;         % [m/s]
    g = 9.81;       % [m/s^2]

    % dynamics
    dx    = zeros(3,1);
    dx(1) = V * sin(chi);            % dx/dt
    dx(2) = V * cos(chi);            % dy/dt
    dx(3) = (g/V) * tan(phiCmd);     % dchi/dt
end

function phi = guidanceLaw(x, guidanceParams)
    % A simple "look-ahead" guidance: find a target point on the path
    % that is L meters ahead in x, and compute heading needed to get there.
    xPos = x(1);
    yPos = x(2);
    chi  = x(3);

    L   = guidanceParams.lookAheadDist;
    x_p = xPos + L;  % naive approach: look L meters ahead in x
    
    % if x_p beyond path, just clamp to final
    if x_p >= max(guidanceParams.x_path)
        x_p = max(guidanceParams.x_path);
    end

    % get reference y from path
    y_ref = interp1(guidanceParams.x_path, guidanceParams.y_path, x_p, 'linear','extrap');

    % desired heading = angle from current (x,y) to (x_p,y_ref)
    % REMEMBER: heading \chi is measured clockwise from y-axis.
    % Standard "atan2" gives angle from x-axis. We want from y-axis, clockwise.
    dx = x_p - xPos;    % w.r.t plane
    dy = y_ref - yPos;

    % angle from plane to the target point in standard XY:
    %   alpha_std = atan2(dy, dx)  (angle from x-axis, CCW)
    alpha_std = atan2(dy, dx);

    % But \chi=0 means "up" (+y), and \chi positive means rotating clockwise.
    % => we can convert: if alpha_std=0 => heading along +x => that is chi=+90 deg
    %    if alpha_std=+pi/2 => heading along +y => that is chi=0
    % Let's define a conversion:
    %    chi_des =  + (pi/2 - alpha_std)   (then ensure correct sign for clockwise)
    chi_des = (pi/2) - alpha_std;  

    % Wrap chi_des to [-pi, +pi] range for continuity
    chi_des = angleWrap(chi_des);

    % Now do a simple heading hold: phi = Kphi * (chi_des - chi)
    heading_error = angleWrap(chi_des - chi);
    phi = guidanceParams.Kphi * heading_error;
end

function ang = angleWrap(ang)
    % Wrap angle to [-pi, pi] for numerical stability
    ang = mod(ang + pi, 2*pi) - pi;
end
