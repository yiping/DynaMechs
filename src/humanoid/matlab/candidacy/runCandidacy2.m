function hop()
    g=9.8;    m=72.5748; L0=1.0;
    k=16*72*10*21.5;
    
    vx = 4.4;
    
    N = 1000;      % Number of Samples
    tSim = 10;     % Max Sim time
    
    tdPos = [];
    tdAng = pi/6;
    
    tStanceDes = .35*.45; % .35s estimated from paper
    tFlightDes = .35*.55;
    %tStanceDes = 0.2633;
    %tFlightDes = 0.2633/.45*.55;
    
    % for 3.5 m/s
    %tStanceDes=.18;
    %tFlightDes=.16;
    
    % for 2.6 m/s
    %tStanceDes=.255;
    %tFlightDes=.255/.64*(1-.64);
    
    % for 4.4 m/s
    %tStanceDes=.205;
    %tFlightDes=0.3158-.205;
    
    % for 6.0 m/s
    %tStanceDes=0.1463;
    %tFlightDes=0.1463;
    
    %tStanceDes = 0.256307061;
    %tFlightDes = 0.097823578;
    %vx = 3;
    
    %tStanceDes = 0.225872939;
    %tFlightDes = 0.120308035;
    %vx = 3.5;
    
    
    %tStanceDes = 0.202446749;
    %tFlightDes = 0.13371407;
    %vx = 4;
    
    %tStanceDes = 0.183808563;
    %tFlightDes = 0.140642473;
    %vx = 4.5;
    
    %tStanceDes = 0.168594963;
    %tFlightDes = 0.142858747;
    %vx = 5;
    
    tStanceDes = 0.155920276;
    tFlightDes = 0.141643052;
    
    vx = 5.5;
    
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

    function [t_out z_out foot_out tf zf en loTime] = simulatePeriod(t0,params,x0)
        flightOptions = odeset('Events',@flightEvents,'AbsTol',1e-9,'RelTol',1e-9);
        stanceOptions = odeset('Events',@stanceEvents,'AbsTol',1e-9,'RelTol',1e-9); 
        
        xorig = 0;
        if nargin==3
            xorig = x0;
        end
        

        z0 = [xorig vx L0*cos(params(1)) params(2)];  
        k = params(3);
        tdAng = params(1);
        
        
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
    function output = periodResidual(params)
        [t z foot tf zf en loTime] = simulatePeriod(0,params);
        output = [0 0 0 0];
        if length(tf)==0
            output(1) = vx - z(end,2);
            output(2) = params(2) - z(end,4);
            output(3) = loTime - tStanceDes;
            output(4) = (t(end)-loTime) - tFlightDes;
        else
            output(1) = vx - z(end,2);
            output(2) = params(2) - z(end,4);
            
            output(3) = loTime - tStanceDes;
            
            
            output(4) = (tf-loTime) - tFlightDes;
        end
    end
    
    %% Initial guesses
    % z0initial = [0 3.5	L0*cos(tdAng)	-0.424237]'; % Cycle (a) stable run like cycle 
    % z0initial = [0 1.5  L0*cos(tdAng) -1.8]';        % Cycle (b) Unstable high cycle 
    
    % Other Limit Cycles
    % z0initial = [0 1.5  L0*cos(tdAng) -.8]'; % unstable cycle
    % z0initial = [0 3    0.8660   0]';     %  vert unstable cycle low
    % z0initial = [0 .3  L0*cos(tdAng) -.2]'; % another stable cycle X
   
    paramsInitial = [pi/6 -1.459228 16*72*10];
     
    paramsInitial = [.19 -1.2250 6.6683e+04]
    
    %% Find Orbit
    params0 = paramsInitial;
    options = optimset('display','iter');
    [params resnorm res] = lsqnonlin(@periodResidual,params0,[pi/20,-inf,16*72*2],[pi/4,-.1,16*72*100*21.5],options);
    res
    ang = params(1)
    vy = params(2)
    k = params(3)
    [t z foot tf zf] = simulatePeriod(0,params);
    zf
    
    params
    
    [t z foot tf zf] = simulatePeriod(0,params);    
    
    %% Optional Distrubance

    z0 = zf;
    z0(1) = 0;
    x0 = 0;
    %zf = z0;    
    tf = 0;
    foot0 = [z0(1)+L0*sin(tdAng) z0(3)-L0*cos(tdAng)];
    
	J = 5;         % Number of Jumps
    ts = []; zs = []; ens = [];
    
    
    figure(15); clf; hold on; axis equal
    
    %% Simulate Multiple Steps
    foots = [];
    tfs=[];
    los=[];
    for i = 1:J
        
        [t z foot tf zf energy lo] = simulatePeriod(tf,params,x0);
        x0 = zf(1);
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
    
%     tic
%         %% Hopping Animation
% 	figure(2); clf; hold on;
%     hOA = plot([z0(1) z0(3)],[foot0(1) foot0(2)],'r');
%     hA = plot(z0(1),z0(3),'ro');
%     hHist = plot(z0(1),z0(3),'r--');
%     axis equal;
%     axis([0 max(zs(:,1)) 0 max(zs(:,3))]); 
%     set(hA,'markerfacecolor','r');
%     startTime = cputime;
%     for i = 1:1:length(ts)
%         set(hOA,'ydata',[zs(i,3) foots(i,2)],'xdata',[zs(i,1) foots(i,1)]);
%         set(hA,'ydata',zs(i,3),'xdata',zs(i,1));
%         set(hHist,'ydata',zs(1:i,3),'xdata',zs(1:i,1));
%         %while (cputime-startTime) < ts(i)
%             pause(.00001);
%         %end
%     end
%     toc
%     ts(end)


     
    %% Point Mass Trajectory
    plot(zs(:,1),0*zs(:,1),'k');
    axis([0 max(zs(:,1)) 0 max(zs(:,3))]); 
    
    %% Energy Plot    
%     figure(4)
%     title('Energy');
%     plot(ts,ens)
Stiffness = params(3);
vx = vx;
Stance = los(1);
Period = tfs(1);
DutyFactor = Stance/Period/2;
Length = Period*vx;


fprintf(1,'Vy = \t%f\n',vy);
fprintf(1,'T  = \t%f\n',Period);
fprintf(1,'Cad  = \t%f\n',60/Period);
fprintf(1,'t_c = \t%f\n',Stance);
fprintf(1,'DF = \t%f\n',DutyFactor);
fprintf(1,'SL = \t%f\n',Length);
fprintf(1,'px = \t%f\n',L0*sin(ang));
fprintf(1,'py = \t%f\n',-L0*cos(ang));


fprintf(1,'touchDownAngle=\t%f;\n',ang);
fprintf(1,'legSpringConstant = \t%f;\n',Stiffness);
fprintf(1,'forwardVelocity = \t%f;\n',vx);
fprintf(1,'maxSLIPHeight = \t%f;\n',max(zs(:,3)));
fprintf(1,'touchDownLength = \t%f;\n',L0);
fprintf(1,'stanceTime = \t%f;\n',tStanceDes);
fprintf(1,'flightTime = \t%f;\n',tFlightDes);





    
    
end