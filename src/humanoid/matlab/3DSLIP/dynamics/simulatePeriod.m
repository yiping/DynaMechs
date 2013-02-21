function [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams)
        
        m = dynParams.m;
        g = dynParams.g;
        tSim = dynParams.tSim;
        N = dynParams.N;
        tdParams = dynParams.tdParams;
        robotLegLength = dynParams.robotLegLength;

        tspan = linspace(t0,t0+tSim,N);
        state0 = stateInit;
        
        k = dynParams.k1;
        L0 = dynParams.L0;
        tdPos = [0 0 0];
        
        side = dynParams.side;
        stanceTime = 0;
        
        % Simulation Flight
        flightOptions = odeset('Events',@flightEventTouchDown,'AbsTol',1e-6,'RelTol',1e-6);
        
        [T STATE tf statef] = ode45(@flightDynamics,tspan,state0,flightOptions,dynParams);
        vel = STATE(:,4:6);
        en = m*g*STATE(:,3) + 1/2*m*sum(vel.^2,2);
        foot = [STATE(:,1)+robotLegLength*sin(tdParams(1))*cos(tdParams(2)*side), ...
                STATE(:,2)+robotLegLength*sin(tdParams(1))*cos(tdParams(2)*side)+side*dynParams.robotHipDisp,...
                STATE(:,3)-robotLegLength*cos(tdParams(1))];
        
        
        if isfield(dynParams,'plot') 
            figure(1)
            subplot(211)
            hold on
            plot3(STATE(:,1),STATE(:,2),STATE(:,3),'b-.');

            subplot(212)
            hold on
            plot(T,1/2*m*sum(vel.^2,2),'r');
            plot(T,m*g*STATE(:,3),'g-.');
            plot(T,T*0,'b--');
        end
        if isfield(dynParams,'plotAll') 
            figure(2)
            subplot(321)
            hold on
            plot(T,STATE(:,1),'b');
            
            subplot(323)
            hold on
            plot(T,STATE(:,2),'b');
            
            subplot(325)
            hold on
            plot(T,STATE(:,3),'b');
            
            subplot(322)
            hold on
            plot(T,STATE(:,4),'b');
            
            subplot(324)
            hold on
            plot(T,STATE(:,5),'b');
            
            subplot(326)
            hold on
            plot(T,STATE(:,6),'b');
        end
        
        
       
        STATE_out = STATE;
        T_out = T;
        FOOT_out = foot;
        EN_out = en;
        
        if length(tf) == 0
            %'break after flight'
            return;
        end
        %fprintf(1,'FLIGHT Ends at t=%f, (%f,%f,%f)\n',T_out(end),STATE(end,1),STATE(end,2),STATE(end,3));
        
        
        
        % Simulate Stance
        t0 = tf;
        tdPos =[STATE(end,1)+robotLegLength*sin(tdParams(1))*cos(tdParams(2)*side), ...
                STATE(end,2)+robotLegLength*sin(tdParams(1))*sin(tdParams(2)*side)+side*dynParams.robotHipDisp,...
                0];
            
        %%tdPos = [STATE(end,1)+robotLegLength*sin(tdParams(1)) STATE(end,2)+tdParams(2) 0];
        
        dynParams.tdPos = tdPos;
        dynParams.tStanceBegin = t0;
        
        
        tspan = linspace(tf,tf+tSim,N);
        stanceOptions = odeset('Events',@stanceEvents,'AbsTol',1e-6,'RelTol',1e-6); 
        [T STATE tf statef] = ode45(@stanceDynamics,tspan,statef,stanceOptions,dynParams);

        foot = ones(length(T),1)*tdPos;   
        leg = STATE(:,1:3) - ones(length(T),1)*tdPos;
        vel = STATE(:,4:6);
        en = m*g*STATE(:,3) + 1/2 * k * (sqrt(sum(leg.^2,2))-L0).^2+ 1/2*m*sum(vel.^2,2);
        if nargout>=7
            stanceTime = tf-t0;
        end
        
        if isfield(dynParams,'plot') 
            figure(1)
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
        
        if isfield(dynParams,'plotAll') 
            figure(2)
            subplot(321)
            hold on
            plot(T,STATE(:,1),'r');
            
            subplot(323)
            hold on
            plot(T,STATE(:,2),'r');
            
            subplot(325)
            hold on
            plot(T,STATE(:,3),'r');
            
            subplot(322)
            hold on
            plot(T,STATE(:,4),'r');
            
            subplot(324)
            hold on
            plot(T,STATE(:,5),'r');
            
            subplot(326)
            hold on
            plot(T,STATE(:,6),'r');
        end
        
        
                
        T_out = [T_out ; T];
        STATE_out = [STATE_out ; STATE];
        FOOT_out = [ FOOT_out ; foot];
        EN_out = [EN_out ; en];
        
        %fprintf(1,'STANCE Ends at t=%f, (%f,%f,%f)\n',T_out(end),STATE(end,1),STATE(end,2),STATE(end,3));
        
        if length(tf) == 0
            %'break after stance'
            return;
        end

        
        % Simulation Flight
        flightOptions = odeset('Events',@flightEventTOF,'AbsTol',1e-6,'RelTol',1e-6);
         tspan = linspace(tf,tf+tSim,N);
        [T STATE tf statef] = ode45(@flightDynamics,tspan,statef,flightOptions,dynParams);
        vel = STATE(:,4:6);
        en = m*g*STATE(:,3) + 1/2*m*sum(vel.^2,2);
        foot = [STATE(:,1)+robotLegLength*sin(tdParams(1))*cos(tdParams(2)*side), ...
                STATE(:,2)+robotLegLength*sin(tdParams(1))*cos(tdParams(2)*side)+side*dynParams.robotHipDisp,...
                STATE(:,3)-robotLegLength*cos(tdParams(1))];
            
        
        if isfield(dynParams,'plot')
            figure(1)
            subplot(211)
            hold on
            plot3(STATE(:,1),STATE(:,2),STATE(:,3),'b-.');

            subplot(212)
            hold on
            plot(T,1/2*m*sum(vel.^2,2),'r');
            plot(T,m*g*STATE(:,3),'g-.');
            plot(T,T*0,'b--');
        end
        
        if isfield(dynParams,'plotAll') 
            figure(2)
            subplot(321)
            hold on
            plot(T,STATE(:,1),'b');
           
            subplot(323)
            hold on
            plot(T,STATE(:,2),'b');
            
            subplot(325)
            hold on
            plot(T,STATE(:,3),'b');

            subplot(322)
            hold on
            plot(T,STATE(:,4),'b');
            
            subplot(324)
            hold on
            plot(T,STATE(:,5),'b');
            
            subplot(326)
            hold on
            plot(T,STATE(:,6),'b');
        end
        
       
        T_out = [T_out ; T];
        STATE_out = [STATE_out ; STATE];
        FOOT_out = [ FOOT_out ; foot];
        EN_out = [EN_out ; en];
        
    end

    