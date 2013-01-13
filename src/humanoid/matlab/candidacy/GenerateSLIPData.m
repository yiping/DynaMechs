function hop()
    g=9.8;    m=72.5748; L0=.96;
    k=16*72*10*21.5;
    
    vx = 4.4;
    
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
    
   
    fid = fopen('SlipData.txt','w');
   
    paramsInitial = [.3 -1.2250 12000];
    for vx = 3:.05:5.5
        
        tStanceDes =10^-0.2*vx^-0.82;
        cad = 2.551*vx*vx-8.8*vx+172.87;
        tFlightDes = 60/cad - tStanceDes;

        %% Find Orbit
        params0 = paramsInitial;
        options = optimset('display','iter');
        [params resnorm res] = lsqnonlin(@periodResidual,params0,[pi/20,-inf,16*72*2],[pi/4,0,16*72*100*21.5],options);
        paramsInitial = params;
        res
        ang = params(1);
        vy = params(2);
        k = params(3);
        [t z foot tf zf energy lo] = simulatePeriod(0,params);


        %% Point Mass Trajectory
        %plot(z(:,1),0*z(:,1),'k');
        %axis([0 max(z(:,1)) 0 max(z(:,3))]); 

        Stiffness = params(3);
        Stance = lo;
        Period = tf;
        DutyFactor = Stance/Period/2;
        Length = Period*vx;
        maxSLIPHeight = max(z(:,3));
        
        fprintf(fid,'%f\t %f\t %f\t %f\t %f\t %f\t %f\n',vx,tStanceDes,tFlightDes,Stiffness, ang, maxSLIPHeight, L0);

%         fprintf(1,'Vy = \t%f\n',vy);
%         fprintf(1,'T  = \t%f\n',Period);
%         fprintf(1,'Cad  = \t%f\n',60/Period);
%         fprintf(1,'t_c = \t%f\n',Stance);
%         fprintf(1,'DF = \t%f\n',DutyFactor);
%         fprintf(1,'SL = \t%f\n',Length);
%         fprintf(1,'px = \t%f\n',L0*sin(ang));
%         fprintf(1,'py = \t%f\n',-L0*cos(ang));
% 
% 
%         fprintf(1,'touchDownAngle=\t%f;\n',ang);
%         fprintf(1,'legSpringConstant = \t%f;\n',Stiffness);
%         fprintf(1,'forwardVelocity = \t%f;\n',vx);
%         fprintf(1,'maxSLIPHeight = \t%f;\n',max(z(:,3)));
%         fprintf(1,'touchDownLength = \t%f;\n',L0);
%         fprintf(1,'stanceTime = \t%f;\n',tStanceDes);
%         fprintf(1,'flightTime = \t%f;\n',tFlightDes);
    end
    fclose(fid);





    
    
end