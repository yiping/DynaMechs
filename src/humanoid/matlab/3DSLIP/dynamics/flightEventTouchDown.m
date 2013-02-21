    function [value isterminal direction] = flightEventTouchDown(t,state,dynParams)
        
        value = state(3)-dynParams.heightThreshold;
        isterminal = 1;	direction = -1;
    end