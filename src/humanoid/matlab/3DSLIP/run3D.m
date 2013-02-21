function hop()
    addpath dynamics
    dynParams.g=9.8;   dynParams.m=72.5748; 
    
    robotLegLength = .97;
    robotHipDisp = .1271;
    dynParams.robotLegLength = robotLegLength;
    dynParams.robotHipDisp  = robotHipDisp;
        
        
    
    
    k=16*72*10*21.5;
    
    dynParams.N = 1000;      % Number of Samples
    dynParams.tSim = 2;     % Max Sim time
    
    
    tStanceDes = .35*.45; % .35s estimated from paper
    tFlightDes = .35*.55;
    vx = 5;
    
    tStanceDes =10^-0.2*vx^-0.82;
        cad = 2.551*vx*vx-8.8*vx+172.87;
        tFlightDes = 60/cad - tStanceDes;
        
        
    figure(1)
    clf;
    hold on;
    global zs foots
    
    function output = periodResidual(params)
        stateInit = zeros(6,1);     
        dynParams.tdParams = [params(1) 0];
        dynParams.heightThreshold = robotLegLength*cos(dynParams.tdParams(1));
        dynParams.L0 = sqrt(robotLegLength^2+robotHipDisp^2 +...
                                2*robotLegLength*robotHipDisp*sin(dynParams.tdParams(1))*sin(dynParams.tdParams(2)));
        dynParams.k1 = params(2);
        dynParams.k2 = dynParams.k1;        
        dynParams.side = 1;
        
        stateInit(3) = params(3);
        stateInit(4) = vx;
        stateInit(5) = params(4);
        t0=0;
        
        [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
        
        output = zeros(5,1);
        output(1) = stanceTime - tStanceDes;
        output(2) = (T_out(end)-stanceTime) - tFlightDes;
        
        output(3) = STATE_out(end,4) - vx;
        output(4) = STATE_out(end,5) + stateInit(5);
    end


    
    
   
    
    %% Find Orbit
    %params0 = paramsInitial;
    %options = optimset('display','iter');
    %tic
    %[params resnorm res] = lsqnonlin(@periodResidual,params0,[pi/20,-inf,16*72*2],[pi/4,-.1,16*72*100*21.5],options);

    tdParams = [];
    
    params0 = [0.2802 2.6721e+04 0.9776 0.1613]+[.05 1e3 .06 -.06];
    
    %output = periodResidual(params0)

    %return
    options = optimset('display','iter');
    [params resnorm res] = lsqnonlin(@periodResidual,params0,[0,1000,.5,0],[pi/4,150000,2,1],options);
    


    
    fprintf(1,'thisStep.touchDownAngle1=\t%f;\n',params(1));
    fprintf(1,'thisStep.touchDownAngle2=\t%f;\n',0);
    
    fprintf(1,'thisStep.k = \t%f;\n',params(2));
    fprintf(1,'thisStep.vx0 = \t%f;\n',vx);
    fprintf(1,'thisStep.vy0 = \t%f;\n',params(4));
    
    fprintf(1,'thisStep.h0 = \t%f;\n',params(3));
    fprintf(1,'thisStep.touchDownLength = \t%f;\n',robotLegLength);
    fprintf(1,'thisStep.stanceTime = \t%f;\n',tStanceDes);
    fprintf(1,'thisStep.flightTime = \t%f;\n',tFlightDes);
    
    
    J = 15;
    
    stateInit = zeros(6,1);     
    
    
    dynParams.tdParams = [params(1) 0];;
    dynParams.heightThreshold = robotLegLength*cos(dynParams.tdParams(1));
    dynParams.L0 = sqrt(robotLegLength^2+robotHipDisp^2 ...
                                + 2*robotLegLength*robotHipDisp*sin(dynParams.tdParams(1))*sin(dynParams.tdParams(2)));
    dynParams.k1 = params(2);
    dynParams.k2 = dynParams.k1;

    stateInit(3) = params(3);
    stateInit(4) = vx;
    stateInit(5) = params(4);
    t0=0;
    dynParams.side = 1;
    
    %stateInit= stateInit+rand(6,1).*[0,0,.01,.02,.01,.01]';

    [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
    fprintf(1,'thisStep.stepWidth = \t%f;\n',tdPos(2));
    
    
    %dynParams.plot = 1;
    dynParams.plotAll = 1;
    
    E_tot = [];
    T_tot = [];
    for i=1:J 

        [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
        dynParams.side = dynParams.side*-1;
        E_tot = [E_tot; EN_out];
        T_tot = [T_tot; T_out];
        
        t0 = tf;
        stateInit = statef;
    end


    



    
    figure(3)
    plot(T_tot,E_tot)
    
    params(1)
    params(2)
    params(3)
    params(4)
    global ssResult
    ssResult = params;
    ssResult(2)=ssResult(2)/1e4;
    
    
end