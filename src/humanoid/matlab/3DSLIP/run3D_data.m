function run3dSLIPData()
    g=9.8;    m=72.5748; L0=.97; robotLegLength = .97;
    k=16*72*10*21.5;
    
    N = 1000;      % Number of Samples
    tSim = 2;     % Max Sim time
        
    tdPos = [0 0 0];
    tdParams = [pi/6 0];
    heightThreshold = 0;
    
    
    tStanceDes = .35*.45; % .35s estimated from paper
    tFlightDes = .35*.55;
    vx = 2;
   
    stepWidthDes = .09;
    
    plotInSim = 0;
    
    figure(1)
    clf;
    hold on;
    global zs foots
    
    function dState = flightDynamics(t,state)
      dState = 0*state;
      dState(1:3) = state(4:6);
      dState(6)   = -g;
    end

    function [value isterminal direction] = flightEventTouchDown(t,state)
        value = state(3)-heightThreshold;
        isterminal = 1;	direction = -1;
    end

    function [value isterminal direction] = flightEventTOF(t,state)
        value = state(6);
        isterminal = 1;	direction = -1;
    end


    function dState  = stanceDynamics(t,state)
        dState = 0*state;
        dState(1:3) = state(4:6);
        relPos = state(1:3)' - tdPos;
        len = norm(relPos);
        relPos = relPos/len;
        force = (L0-len)*k;
        dState(4:6) = relPos*force/m;
        dState(6) = dState(6) - g;
        %state
        %dState
    end

    function [value isterminal direction] = stanceEvents(t,state)
        relPos = state(1:3)' - tdPos;
        len = norm(relPos);
        value = len-L0;
        %value = 1;
        isterminal = 1;	direction = +1;
    end

    function [T_out STATE_out FOOT_out tf statef EN_out stanceTime] = simulatePeriod(t0,stateInit)
        tspan = linspace(t0,t0+tSim,N);
        state0 = stateInit;
        
        % Simulation Flight
        flightOptions = odeset('Events',@flightEventTouchDown,'AbsTol',1e-9,'RelTol',1e-9);
        
        [T STATE tf statef] = ode45(@flightDynamics,tspan,state0,flightOptions);
        vel = STATE(:,4:6);
        en = m*g*STATE(:,3) + 1/2*m*sum(vel.^2,2);
        foot = [STATE(:,1)+robotLegLength*sin(tdParams(1)) STATE(:,2)+tdParams(2)  STATE(:,3)-robotLegLength*cos(tdParams(1))];
        
        if plotInSim
            subplot(211)
            hold on
            plot3(STATE(:,1),STATE(:,2),STATE(:,3),'b-.');

            subplot(212)
            hold on
            plot(T,1/2*m*sum(vel.^2,2),'r');
            plot(T,m*g*STATE(:,3),'g-.');
            plot(T,T*0,'b--');
        end
        
        STATE_out = STATE;
        T_out = T;
        FOOT_out = foot;
        EN_out = en;
        
        stanceTime =0;
        if length(tf) == 0
            %'break after flight'
            return;
        end
        %fprintf(1,'FLIGHT Ends at t=%f, (%f,%f,%f)\n',T_out(end),STATE(end,1),STATE(end,2),STATE(end,3));
        
        
        
        % Simulate Stance
        t0 = tf;
        tdPos = [STATE(end,1)+robotLegLength*sin(tdParams(1)) STATE(end,2)+tdParams(2) 0];
        
        tspan = linspace(tf,tf+tSim,N);
        stanceOptions = odeset('Events',@stanceEvents,'AbsTol',1e-9,'RelTol',1e-9); 
        [T STATE tf statef] = ode45(@stanceDynamics,tspan,statef,stanceOptions);

        foot = ones(length(T),1)*tdPos;   
        leg = STATE(:,1:3) - ones(length(T),1)*tdPos;
        vel = STATE(:,4:6);
        en = m*g*STATE(:,3) + 1/2 * k * (sqrt(sum(leg.^2,2))-L0).^2+ 1/2*m*sum(vel.^2,2);
        if nargout>=7
            stanceTime = tf-t0;
        end
        
        if plotInSim
            
            subplot(211)
            hold on
            plot3(STATE(:,1),STATE(:,2),STATE(:,3),'r');
            plot3([STATE(1,1),tdPos(1)], [STATE(1,2),tdPos(2)], [STATE(1,3) tdPos(3)],'k--');
            plot3([STATE(end,1),tdPos(1)], [STATE(end,2),tdPos(2)], [STATE(end,3) tdPos(3)] ,'k--');

            subplot(212)
            hold on

            plot(T,1/2*m*sum(vel.^2,2),'r');
            plot(T,m*g*STATE(:,3),'g-.');
            plot(T,1/2 * k * (sqrt(sum(leg.^2,2))-L0).^2,'b--');
        end
        
        T_out = [T_out ; T];
        STATE_out = [STATE_out ; STATE];
        FOOT_out = [ FOOT_out ; foot];
        EN_out = [EN_out ; en];
        
        %fprintf(1,'STANCE Ends at t=%f, (%f,%f,%f)\n',T_out(end),STATE(end,1),STATE(end,2),STATE(end,3));
        
        if length(tf) == 0
           % 'break after stance'
            return;
        end

        
        % Simulation Flight
        flightOptions = odeset('Events',@flightEventTOF,'AbsTol',1e-9,'RelTol',1e-9);
         tspan = linspace(tf,tf+tSim,N);
        [T STATE tf statef] = ode45(@flightDynamics,tspan,statef,flightOptions);
        vel = STATE(:,4:6);
        en = m*g*STATE(:,3) + 1/2*m*sum(vel.^2,2);
        foot = [STATE(:,1)+robotLegLength*sin(tdParams(1)) STATE(:,2)+tdParams(2)  STATE(:,3)-robotLegLength*cos(tdParams(1))];
        
        if plotInSim
            
            subplot(211)
            hold on
            plot3(STATE(:,1),STATE(:,2),STATE(:,3),'b-.');

            subplot(212)
            hold on
            plot(T,1/2*m*sum(vel.^2,2),'r');
            plot(T,m*g*STATE(:,3),'g-.');
            plot(T,T*0,'b--');
        end
        
        T_out = [T_out ; T];
        STATE_out = [STATE_out ; STATE];
        FOOT_out = [ FOOT_out ; foot];
        EN_out = [EN_out ; en];
        
    end

    function output = periodResidual(params)
        stateInit = zeros(6,1);     
        tdParams = params(1:2);
        heightThreshold = robotLegLength*cos(tdParams(1));
        L0 = sqrt(robotLegLength^2+tdParams(2)^2);
        k = params(3);
        
        stateInit(3) = params(4);
        stateInit(4) = vx;
        stateInit(5) = params(5);
        t0=0;
        
        [T_out STATE_out FOOT_out tf statef EN_out stanceTime] = simulatePeriod(t0,stateInit);
        
        output = zeros(5,1);
        output(1) = stanceTime - tStanceDes;
        output(2) = (T_out(end)-stanceTime) - tFlightDes;
        
        output(3) = STATE_out(end,4) - vx;
        output(4) = STATE_out(end,5) + stateInit(5);
        output(5) = tdPos(2) - stepWidthDes;
        
    end
    
   
    
    %% Find Orbit
    %params0 = paramsInitial;
    %options = optimset('display','iter');
    %tic
    %[params resnorm res] = lsqnonlin(@periodResidual,params0,[pi/20,-inf,16*72*2],[pi/4,-.1,16*72*100*21.5],options);

    tdParams = [];
    
    params0 = [0.320424 .05 15207.650564 1.25 .2];
    
    global paramSet
    paramSet = [];
    %output = periodResidual(params)
    for vx = 3:.05:5.5  
        tStanceDes =10^-0.2*vx^-0.64;
        cad = 2.551*vx*vx-8.8*vx+172.87;
        tFlightDes = 60/cad - tStanceDes;
        options = optimset('display','none');
        [params resnorm res exitflag output] = lsqnonlin(@periodResidual,params0,[0,0,1000,.5,0],[pi/4,1,150000,2,1],options);
        
        fprintf(1,'%f \t %d\n',vx,output.iterations);
        params0 = params;
        paramSet  = [paramSet ; params];
    end
        
        
    params
    close all

    
    J = 15;
    
    stateInit = zeros(6,1);     
    tdParams = params(1:2);
    heightThreshold = robotLegLength*cos(tdParams(1));
    L0 = sqrt(robotLegLength^2+tdParams(2)^2);
    k = params(3);

    stateInit(3) = params(4);
    stateInit(4) = vx;
    stateInit(5) = params(5);
    t0=0;
    %stateInit= stateInit+rand(6,1).*[0,0,.01,.02,.01,.01]';

    E_tot = [];
    T_tot = [];
    
    ts = []; States = []; ens = [];
    foots = [];
    tfs=[];
    los=[];
    
    
    for i=1:J 
        [T_out STATE_out FOOT_out tf statef EN_out stanceTime] = simulatePeriod(t0,stateInit);
        tdParams(2) = tdParams(2)*-1;
        E_tot = [E_tot; EN_out];
        T_tot = [T_tot; T_out];
        
        tfs=[tfs tf];
        los=[los stanceTime];
        ts = [ts ; T_out]; States = [States ; STATE_out]; foots = [foots ; FOOT_out];
        
        t0 = tf;
        stateInit = statef;
    end
        
    
    foot0 = [stateInit(1)+L0*sin(tdParams(1)) stateInit(2)+tdParams(2) stateInit(3)-L0*cos(tdParams(1))];
    
        tic
        %% Hopping Animation
	figure(2); clf; hold on;
    hOA = plot3([stateInit(1) stateInit(2)],[foot0(1) foot0(2)],[ stateInit(3) foot0(3)],'r');
    hA = plot3(stateInit(1),stateInit(2),stateInit(3),'ro');
    hHist = plot3(stateInit(1),stateInit(2),stateInit(3),'r--');
    
    axis([0 max(States(:,1))  min(foots(:,2)) max(foots(:,2)) 0 max(States(:,3))]); 
    set(hA,'markerfacecolor','r');
    startTime = cputime;
    for i = 1:4:length(ts)
        set(hOA,'zdata',[States(i,3) foots(i,3)],'ydata',[States(i,2) foots(i,2)],'xdata',[States(i,1) foots(i,1)]);
        set(hA,'zdata',States(i,3),'ydata',States(i,2),'xdata',States(i,1));
        set(hHist,'zdata',States(1:i,3),'ydata',States(1:i,2),'xdata',States(1:i,1));
        %while (cputime-startTime) < ts(i)
            pause(.00001);
        %end
    end
    toc
    ts(end)



    
    figure(3)
    plot(T_tot,E_tot)
    
    
end