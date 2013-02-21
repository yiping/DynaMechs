function hop()
    global ssResult

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
    
    vx0 = 3.4;
    vxf = 3.5;
    vy0  = 0.313;
    vyf  = -ssResult(4);
    h0   = 0.997582;
    hf   = ssResult(3);
    
%     vxf= 3.500000;
%     vx0=3.600000;
%     vyf=-0.161856;
%     vy0=0.100000;
     k0=2.6745;
     k0 = ssResult(2);
%     
%     h0=0.900000;	 
%     hf=0.910240;
    tStanceDes = 0.2259;
    
  
    
    
    figure(1)
    clf;
    hold on;
    global zs foots
    
    function output = periodResidual(params)
        stateInit = zeros(6,1);     
        dynParams.tdParams = [params(1) params(2)];
        dynParams.heightThreshold = robotLegLength*cos(dynParams.tdParams(1));
        dynParams.L0 = sqrt(robotLegLength^2+robotHipDisp^2 +...
                                2*robotLegLength*robotHipDisp*sin(dynParams.tdParams(1))*sin(dynParams.tdParams(2)));
        %dynParams.k1 = params(3)*1e4;
        %dynParams.k2 = params(4)*1e4;        
        dynParams.k1 = (k0+params(3))*1e4;
        dynParams.k2 = (k0-params(3))*1e4;        

        dynParams.side = 1;
        
        stateInit(3) = h0;
        stateInit(4) = vx0;
        stateInit(5) = vy0;
        t0=0;
        
        [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
        
        output = zeros(5,1);
        %output(1) = stanceTime - tStanceDes;
        output(2) = STATE_out(end,3) - hf;
        
        output(3) = STATE_out(end,4) - vxf;
        output(4) = STATE_out(end,5) - vyf;
    end


    
    
   
    
    %% Find Orbit
    %params0 = paramsInitial;
    %options = optimset('display','iter');
    %tic
    %[params resnorm res] = lsqnonlin(@periodResidual,params0,[pi/20,-inf,16*72*2],[pi/4,-.1,16*72*100*21.5],options);

    tdParams = [];
    
    params0 = [0.3 0 0];%2.6721e+04/1e4/8 2.6721e+04/1e4/8];
    
    %dynParams.plot = 1;
    %output = periodResidual(params0)

    %return
    options = optimset('display','iter');
    %[params resnorm res] = lsqnonlin(@periodResidual,params0,[0,-pi/4,1000/1e4,1000/1e4],[pi/4,pi/4,150000/1e4,150000/1e4],options);
    [params resnorm res] = lsqnonlin(@periodResidual,params0,[0,-pi/4,-k0],[pi/4,pi/4,k0],options)
    
    nonssParams = params
    nonssInit = [h0 vx0 vy0]';
    
    
    
    J = 15;
    
    stateInit = zeros(6,1);     
    
    
    
    dynParams.side = 1;
    dynParams.k0 = ssResult(2);

    stateInit(3:5) = nonssInit;
    
    [Js_nss Ju_nss] = evalJacobian2(stateInit, nonssParams, dynParams);
    K_nss = - Ju_nss\Js_nss;
    
    ssParams = [ssResult(1) 0 0];
    ssInit = [ssResult(3) vxf ssResult(4)]';
    stateInit(3:5) = ssInit;
    
    [Js_ss Ju_ss] = evalJacobian2(stateInit, ssParams, dynParams);
    K_ss = - Ju_ss\Js_ss;
    
    
    
%     stateInit(3:5) = ssInit+ [0 0 .1]';
%     params = ssParams+(K_ss*(stateInit(3:5)-ssInit))';
%     
%     dynParams.tdParams = [params(1) params(2)];
%     dynParams.heightThreshold = robotLegLength*cos(dynParams.tdParams(1));
%     dynParams.L0 = sqrt(robotLegLength^2+robotHipDisp^2 +...
%                         2*robotLegLength*robotHipDisp*sin(dynParams.tdParams(1))*sin(dynParams.tdParams(2)));
%     dynParams.k1 = (k0+params(3))*1e4;
%     dynParams.k2 = (k0-params(3))*1e4;   
% 
% 
%     %dynParams
%     
%     [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
%     
%     statef
%     return
%     
%     
%     stateInit = statef;
%     dynParams.side = -1;
%     
%     params = ssParams;
%     
%     stateInit
%     
%     
%     
%     dynParams.tdParams = [params(1) params(2)];
%     dynParams.heightThreshold = robotLegLength*cos(dynParams.tdParams(1));
%     dynParams.L0 = sqrt(robotLegLength^2+robotHipDisp^2 +...
%                         2*robotLegLength*robotHipDisp*sin(dynParams.tdParams(1))*sin(dynParams.tdParams(2)));
%     dynParams.k1 = (k0+params(3))*1e4;
%     dynParams.k2 = (k0-params(3))*1e4;   
% 
% 
%     dynParams
%     
%     [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
%    
%     statef
%     
%     return
    

        
    close all
    dynParams.plotAll = 1;
    dynParams.plot = 1;
    
    
    stateInit(3:5) = nonssInit+[.05 -.6 .2]';    
    t0=0;
    dynParams.side = 1;
    
    %stateInit= stateInit+rand(6,1).*[0,0,.01,.02,.01,.01]';

    E_tot = [];
    T_tot = [];
    stateInit
    for i=1:J 
        
        controlState = stateInit(3:5).*[1 1 dynParams.side]';
        if i==1
            params = nonssParams + (K_nss*(controlState-nonssInit))';
        else
            params = ssParams + (K_ss*(controlState-ssInit))';
        end
        
        
        dynParams.tdParams = [params(1) params(2)];
        dynParams.heightThreshold = robotLegLength*cos(dynParams.tdParams(1));
        dynParams.L0 = sqrt(robotLegLength^2+robotHipDisp^2 +...
                            2*robotLegLength*robotHipDisp*sin(dynParams.tdParams(1))*sin(dynParams.tdParams(2)));
        dynParams.k1 = (k0+params(3))*1e4;
        dynParams.k2 = (k0-params(3))*1e4;   
    
    
        [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
        dynParams.side = dynParams.side*-1;
        
        E_tot = [E_tot; EN_out];
        T_tot = [T_tot; T_out];
        
        statef
        
        t0 = tf;
        stateInit = statef';
    end
    
    STATE_out(end,:)
    
        
    
    
    %figure(3)
    %plot(T_tot,E_tot)
    
    
end



% t0=0;
%     %stateInit= stateInit+rand(6,1).*[0,0,.01,.02,.01,.01]';
% 
%     E_tot = [];
%     T_tot = [];
%     
%     ts = []; States = []; ens = [];
%     foots = [];
%     tfs=[];
%     los=[];
%     
%     
%     for i=1:J 
%         [T_out STATE_out FOOT_out tf statef EN_out stanceTime] = simulatePeriod(t0,stateInit);
%         tdParams(2) = tdParams(2)*-1;
%         E_tot = [E_tot; EN_out];
%         T_tot = [T_tot; T_out];
%         
%         tfs=[tfs tf];
%         los=[los stanceTime];
%         ts = [ts ; T_out]; States = [States ; STATE_out]; foots = [foots ; FOOT_out];
%         
%         t0 = tf;
%         stateInit = statef;
%     end
%         
%     
%     foot0 = [stateInit(1)+L0*sin(tdParams(1)) stateInit(2)+tdParams(2) stateInit(3)-L0*cos(tdParams(1))];
%     
%         tic
%         %% Hopping Animation
% 	figure(2); clf; hold on;
%     hOA = plot3([stateInit(1) stateInit(2)],[foot0(1) foot0(2)],[ stateInit(3) foot0(3)],'r');
%     hA = plot3(stateInit(1),stateInit(2),stateInit(3),'ro');
%     hHist = plot3(stateInit(1),stateInit(2),stateInit(3),'r--');
%     
%     axis([0 max(States(:,1))  min(foots(:,2)) max(foots(:,2)) 0 max(States(:,3))]); 
%     set(hA,'markerfacecolor','r');
%     startTime = cputime;
%     for i = 1:4:length(ts)
%         set(hOA,'zdata',[States(i,3) foots(i,3)],'ydata',[States(i,2) foots(i,2)],'xdata',[States(i,1) foots(i,1)]);
%         set(hA,'zdata',States(i,3),'ydata',States(i,2),'xdata',States(i,1));
%         set(hHist,'zdata',States(1:i,3),'ydata',States(1:i,2),'xdata',States(1:i,1));
%         %while (cputime-startTime) < ts(i)
%             pause(.00001);
%         %end
%     end
%     toc
%     ts(end)