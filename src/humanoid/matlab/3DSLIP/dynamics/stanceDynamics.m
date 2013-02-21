 function dState  = stanceDynamics(t,state,dynParams)
        
        dState = 0*state;
        dState(1:3) = state(4:6);
        
        if isfield(dynParams,'CoPVel')
            CoP = dynParams.tdPos + dynParams.CoPInitOffset  + dynParams.CoPVel*(t-dynParams.tStanceBegin);            
            relPos = state(1:3)' - CoP;
        else
            relPos = state(1:3)' - dynParams.tdPos;
        end
        
        
        
        if dot(state(4:6),relPos)<0
            k = dynParams.k1;
        else
            k = dynParams.k2;
        end
        
        
        len = norm(relPos);
        relPos = relPos/len;
        force = (dynParams.L0-len)*k;
        dState(4:6) = relPos*force/dynParams.m;
        dState(6) = dState(6) - dynParams.g;
        %state
        %dState
    end