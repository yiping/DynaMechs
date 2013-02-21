function hop()
    addpath dynamics
    dynParams.g=9.8;   dynParams.m=72.5748; 
    
    robotLegLength = .97;
    robotHipDisp = .09;
    dynParams.robotLegLength = robotLegLength;
    dynParams.robotHipDisp  = robotHipDisp;
        
        
    
    
    k=16*72*10*21.5;
    
    dynParams.N = 1000;      % Number of Samples
    dynParams.tSim = 2;     % Max Sim time
    
    
    tStanceDes = .35*.45; % .35s estimated from paper
    
    vx0 = 3.4;
    vxf = 3.5;
    vy0  = 0.313;
    vyf  = -0.1613;
    h0   = 0.9976;
    hf   = 0.9776;
    vx   = 3.5;
    
    
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
        dynParams.k1 = (dynParams.k0+params(3))*1e4;
        dynParams.k2 = (dynParams.k0-params(3))*1e4;        
        dynParams.side = 1;
        
        stateInit(3) = h0;
        stateInit(4) = vx0;
        stateInit(5) = vy0;
        t0=0;
        
        [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
        
        output = zeros(4,1);
        %output(4) = stanceTime - tStanceDes;
        output(1) = STATE_out(end,3) - hf;
        
        output(2) = STATE_out(end,4) - vxf;
        output(3) = STATE_out(end,5) - vyf;
    end


    function output = periodResidualSS(params)
        stateInit = zeros(6,1);     
        dynParams.tdParams = [params(1) 0];
        dynParams.heightThreshold = robotLegLength*cos(dynParams.tdParams(1));
        dynParams.L0 = sqrt(robotLegLength^2+robotHipDisp^2 +...
                                2*robotLegLength*robotHipDisp*sin(dynParams.tdParams(1))*sin(dynParams.tdParams(2)));
        dynParams.k1 = params(2)*1e4;
        dynParams.k2 = dynParams.k1;        
        dynParams.side = 1;
        
        stateInit(3) = params(3);
        stateInit(4) = vx;
        stateInit(5) = params(4);
        t0=0;
        
        [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
        
        output = zeros(4,1);
        output(1) = stanceTime - tStanceDes;
        output(2) = (T_out(end)-stanceTime) - tFlightDes;
        
        output(3) = STATE_out(end,4) - vx;
        output(4) = STATE_out(end,5) + stateInit(5);
    end

    
   
    
    VX  = 3.5:.5:5.5;
    EVX = -.5:.1:.5;
    VY  = -.1:.1:.4;
    H   = .9:.05:1.1;
    
    numRules = length(VX)*length(EVX)*length(VY)*length(H);
    numMFs     = [length(H) length(VY) length(EVX)  length(VX)];
    numRules
    
    params0 = [0.2802 2.6721 0.9776 0.1613];
    
    global VXresults VXfb VXparams VXinits Vstiffs 
    VXresults = []
    VXfb = cell(length(VX),1);
    VXparams =cell(length(VX),1);
    VXinits = cell(length(VX),1);
    Vstiffs = zeros(length(VX),1);
    
    for i=1:length(VX)
        vx = VX(i);
        tStanceDes =10^-0.2*vx^-0.82;
        cad = 2.551*vx*vx-8.8*vx+172.87;
        tFlightDes = 60/cad - tStanceDes;
    
        options = optimset('display','none');
        [params resnorm res] = lsqnonlin(@periodResidualSS,params0,[0,.5,.5,0],[pi/4,150,2,1],options);
        VXresults = [VXresults ; params];
        params0 = params;
        
        ssParams = [params(1) 0 0];
        ssInit = [params(3) vx params(4)]';
        VXparams{i} = ssParams;
        VXinits{i} = ssInit;
        
        Vstiffs(i) = params(2);
        
        stateInit(3:5) = ssInit;
        dynParams.k0 = params(2);
        
        stateInit = zeros(6,1);
        stateInit(3:5) = ssInit;
        
        [Js_ss Ju_ss] = evalJacobian2(stateInit, ssParams, dynParams);
        K_ss = - Ju_ss\Js_ss;
        VXfb{i} = K_ss;
        
    end
    
    
    maxRes = 0;
    global ruleParams
    global ruleFeedback
    ruleParams = [];
    ruleFeedback = cell(numRules,1);
    for i=214:numRules
        ruleNum = i-1;
        mfIndices = zeros(4,1);
        for j=1:4;
           mfIndex = mod(ruleNum, numMFs(j));
           ruleNum = (ruleNum-mfIndex)/numMFs(j);
           mfIndices(j) = mfIndex+1; 
        end
        
        h0 = H(mfIndices(1));
        vy0 = VY(mfIndices(2));
        evx = EVX(mfIndices(3));
        vxf = VX(mfIndices(4));
        vx0 = vxf + evx;
        
        hf = VXresults(mfIndices(4),3);
        vyf = -VXresults(mfIndices(4),4);

        %params0 = [0.7802 0 2.6721e+04/1e4/8 2.6721e+04/1e4/8];
    
        params0 = VXparams{mfIndices(4)}+(VXfb{mfIndices(4)}*([h0 vx0 vy0]' - VXinits{mfIndices(4)}))';
        

        while robotLegLength*cos(params0(1)) > h0
            params0(1) = params0(1)+.1;
        end
        
%         dynParams.plot = 1;
%         output = periodResidual(params0)
        dynParams.k0 = Vstiffs(mfIndices(4));
        
        options = optimset('display','none','TolFun',1e-9,'TolX',1e-9,'MaxFunEvals',500);
        [params resnorm res] = lsqnonlin(@periodResidual,params0,[0,-pi/4,-15],[pi/4,pi/4,15],options);

        if resnorm > maxRes
            maxRes = resnorm;
        end
         
        stateInit = zeros(6,1);
        stateInit(3) = h0;
        stateInit(4) = vx0;
        stateInit(5) = vy0;
        
        [Js_nss Ju_nss] = evalJacobian2(stateInit, params, dynParams);
        K_nss = - Ju_nss\Js_nss;
    
        ruleParams = [ruleParams ; params];
        ruleFeedback{i} = K_nss;
        
         fprintf('i=%d  vxf= %.2f vx0=%.2f vyf=%.2f vy0=%.2f h0=%.3f hf=%.3f res=%e v %e\n',i,vxf, vx0, vyf, vy0, h0,hf,resnorm,maxRes);
%         options = optimset('display','iter','TolFun',1e-9);
%         %params0 = [VXresults(mfIndices(4),1) 0 VXresults(mfIndices(4),2) VXresults(mfIndices(4),2)];
%         params0 = [0.7802 0 2.6721e+04/1e4/8 2.6721e+04/1e4/8];
%         [params resnorm res] = lsqnonlin(@periodResidual,params0,[0,-pi/4,1000/1e4,1000/1e4],[pi/4,pi/4,150000/1e4,150000/1e4],options);
%          res
%          norm(res)
         
%          [Js Ju] = evalJacobian2(stateInit, params, dynParams);
%          dParams = - (Ju\res)'
%          
% 
%          params0 = params - (Ju\res)'*.1;
%          [params resnorm res] = lsqnonlin(@periodResidual,params0,[0,-pi/4,1000/1e4,1000/1e4],[pi/4,pi/4,150000/1e4,150000/1e4],options);
%          
%          
%          dynParams.plot = 1;
%          periodResidual(params)
%          return
          
         
         
         
%         %params
%         
%         stateInit = zeros(6,1);     
%         dynParams.tdParams = [params(1) params(2)];
%         dynParams.heightThreshold = robotLegLength*cos(dynParams.tdParams(1));
%         dynParams.L0 = sqrt(robotLegLength^2+robotHipDisp^2 +...
%                                 2*robotLegLength*robotHipDisp*sin(dynParams.tdParams(1))*sin(dynParams.tdParams(2)));
%         dynParams.k1 = params(3)*1e4;
%         dynParams.k2 = params(4)*1e4;        
%         dynParams.side = 1;
%         
%         stateInit(3) = h0;
%         stateInit(4) = vx0;
%         stateInit(5) = vy0;
%         t0=0;
%         
%         [T_out STATE_out FOOT_out tf statef EN_out stanceTime tdPos] = simulatePeriod(t0,stateInit,dynParams);
% 
%         
%         stanceTime
%         tStanceDes
        
        return

    end    
    
    
    
   
    
    
end



