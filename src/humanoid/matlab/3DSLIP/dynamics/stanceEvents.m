function [value isterminal direction] = stanceEvents(t,state,dynParams)
        relPos = state(1:3)' - dynParams.tdPos;
        len = norm(relPos);
        value = len-dynParams.L0;
        %value = 1;
        isterminal = 1;	direction = +1;
    end