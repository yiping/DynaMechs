function hop()
    g=9.8;    m=72.5748;	k=16*72*10;   L0=.91;
    N = 1000;      % Number of Samples
    tSim = 10;     % Max Sim time
    
    tdPos = [];
    tdAng = pi/6;
    
    figure(1)
    clf;
    hold on;
    global zs foots
    
    function dz = flightDynamics(t,z)
       dz = z;
       x = z(1); xdot = z(2);
       y = z(3); ydot = z(4);
       dz(1) = xdot;
       dz(2) = 0;
       
       dz(3) = ydot;
       dz(4) = -g;
    end
    function [value isterminal direction] = flightEvents(t,z)
       value = z(3)-L0*cos(tdAng);
       isterminal = 1;	direction = -1;
    end
    function dz  = stanceDynamics(t,z)
       dz = z;
       x = z(1); xdot = z(2);
       y = z(3); ydot = z(4);
       
       len = norm([x-tdPos(1) y]);
       dz(1) = xdot;
       dz(2) = 1/m * ( k * (L0 - len) * (x-tdPos(1)) / len);
       
       dz(3) = ydot;
       dz(4) = -g + 1/m * ( k * (L0 - len) * y / len);
    end
    function [value isterminal direction] = stanceEvents(t,z)
       x = z(1); xdot = z(2);
       y = z(3); ydot = z(4);
       
       len = norm([x-tdPos(1) y]);
       value = len-L0;
       isterminal = 1;	direction = +1;
    end
    function [t_out z_out foot_out tf zf en loTime] = simulatePeriod(t0,z0)
        flightOptions = odeset('Events',@flightEvents,'AbsTol',1e-9,'RelTol',1e-9);
        stanceOptions = odeset('Events',@stanceEvents,'AbsTol',1e-9,'RelTol',1e-9);    
        

        en = [];
        tspan = linspace(t0,t0+tSim,N);
        tdPos = [z0(1)+z0(3)*tan(tdAng) 0];
        %tdPos = [z0(1)+L0*sin(tdAng) z0(3)-L0*cos(tdAng)];
        
        % Simulation Stance
        [t z tf zf] = ode45(@stanceDynamics,tspan,z0,stanceOptions);
        leg = z(:,[1 3]) - ones(length(t),1)*tdPos;
        
        if nargout == 7
            loTime = tf;
        end
                    
        vel = z(:,[2 4]);
        en = m*g*z(:,3) + 1/2 * k * (sqrt(sum(leg.^2,2))-L0).^2 + 1/2*m*sum(vel.^2,2);
        foot = ones(length(z(:,1)),1)*tdPos;
        
        plot(z(:,1),z(:,3),'r');
        %plot(z(:,1),sqrt(L0^2-(z(:,1)-tdPos(1)).^2),'k--')
        
        z_out = z;
        t_out = t;
        foot_out = foot;
        
        if length(tf) == 0
            'break after stance'
            return;
        end
        
        tspan = linspace(tf,tf+tSim,N);
        [t z tf zf] = ode45(@flightDynamics,tspan,zf,flightOptions);
        foot = [z(:,1)+L0*sin(tdAng) z(:,3)-L0*cos(tdAng)];
        
        vel = z(:,[2 4]);
        en2 = m*g*z(:,3) + 1/2*m*(z(:,2).^2 + z(:,4).^2);
        en = [en ; en2];
        
        plot(z(:,1),z(:,3),'b');
        %plot([z(1,1) z(end,1)],[L0*cos(tdAng) L0*cos(tdAng)],'k--')
        
        t_out = [t_out ; t];
        z_out = [z_out ; z];
        foot_out = [ foot_out ; foot]; 
    end
    function output = periodResidual(z0)
        [t z foot tf zf] = simulatePeriod(0,z0);
        if length(tf)==0
            
            output = z0(2:4) - z(end,2:4)';
        else
            output = z0(2:4) - zf(2:4)';
        end
    end
    
    %% Initial guesses
     z0initial = [0 3.5	L0*cos(tdAng)	-0.424237]'; % Cycle (a) stable run like cycle 
    % z0initial = [0 1.5  L0*cos(tdAng) -1.8]';        % Cycle (b) Unstable high cycle 
    
    % Other Limit Cycles
    % z0initial = [0 1.5  L0*cos(tdAng) -.8]'; % unstable cycle
    % z0initial = [0 3    0.8660   0]';     %  vert unstable cycle low
    % z0initial = [0 .3  L0*cos(tdAng) -.2]'; % another stable cycle X
   
     
    %% Find Orbit
    z0 = z0initial;
    options = optimset('display','iter');
    z0 = fsolve(@periodResidual,z0initial,options)
    [t z foot tf zf] = simulatePeriod(0,z0);    
    z0orig = z0;
    
    %% Optional Distrubance
    z0 = z0+[0 0 0 0]';
    
    zf = z0;    tf = 0;
    foot0 = [z0(1)+L0*sin(tdAng) z0(3)-L0*cos(tdAng)];
    
	J = 5;         % Number of Jumps
    ts = []; zs = []; ens = [];
    
    
    figure(1); clf; hold on; axis equal
    
    %% Simulate Multiple Steps
    foots = [];
    tfs=[];
    los=[];
    for i = 1:J
        [t z foot tf zf energy lo] = simulatePeriod(tf,zf);
        tfs=[tfs tf];
        los=[los lo];
        ts = [ts ; t]; zs = [zs ; z]; foots = [foots ; foot];
        ens = [ens ; energy];
        if length(tf) == 0
            'break after sim'
            break;
        end
        %fprintf(1,'%f\t',zf);
        %fprintf(1,'\n');
    end
    
    fprintf(1,'\n');
    fprintf(1,'%f\t',z0);
    fprintf(1,'\n');
     
    %% Point Mass Trajectory
    plot(zs(:,1),0*zs(:,1),'k');
    axis([0 max(zs(:,1)) 0 max(zs(:,3))]); 
    
    %% Energy Plot    
%     figure(4)
%     title('Energy');
%     plot(ts,ens)
    
    %% Jacobian Calculation   
%     figure(2); clf;
%     Jac = zeros(4,4);
%     step = 10^-4.5;
%     ident = eye(4);
%     for i=1:1:4
%         z0 = z0orig + ident(:,i)*step;
%         [t z foot tf zf] = simulatePeriod(0,z0);
%         Jac(:,i)=(zf'-z0orig)/step;
%     end
%     Jac=Jac([2 4],[2 4])
%     [V D]=eig(Jac)
%     eigVals = eig(Jac)
%     normofEigVals = abs(eigVals)
%     
%     %% Hopping Animation
% 	figure(2); clf; hold on;
%     hOA = plot([z0(1) z0(3)],[foot0(1) foot0(2)],'r');
%     hA = plot(z0(1),z0(3),'ro');
%     hHist = plot(z0(1),z0(3),'r--');
%     axis equal;
%     axis([0 max(zs(:,1)) 0 max(zs(:,3))]); 
%     set(hA,'markerfacecolor','r');
%     for i = 1:4:length(ts)
%         set(hOA,'ydata',[zs(i,3) foots(i,2)],'xdata',[zs(i,1) foots(i,1)]);
%         set(hA,'ydata',zs(i,3),'xdata',zs(i,1));
%         set(hHist,'ydata',zs(1:i,3),'xdata',zs(1:i,1));
%         pause(.0001);
%     end
%     
%     
%     %% Code to Draw Vector Field
%     global XV YV XA YA
%     XV = [];
%     YV = [];
%     XA = [];
%     YA = [];
%     i = 1;
%     
%     stepx = .01;
%     stepy = .1;
%     num = 10;
%     for xv = (z0orig(2)-stepx) : stepx/num : (z0orig(2)+stepx)
%     %for xv = 1.45:.01:1.75
%         xv;
%        for yv = (z0orig(4)-stepy) : stepy/num : (z0orig(4)+stepy)
%        %for yv = -.2:-.05:-1.55
%             z0 = [0 ; xv ; L0*cos(tdAng) ; yv];
%             [t z foot tf zf] = simulatePeriod(0,z0);
%             if size(zf) > 0
%                 XV(i) = xv;
%                 YV(i) = yv;
%                 XA(i) = zf(2)-xv;
%                 YA(i) = zf(4)-yv;
%                 i=i+1;5
%             end
%        end
%     end
%     figure(3); clf; hold on;
%     quiver(XV,YV,XA,YA);
%     scatter(z0orig(2),z0orig(4),'r','filled')
%     xlabel('v_x');
%     ylabel('v_y');
   
Stance = los(1)
Period = tfs(1)
    
    
end