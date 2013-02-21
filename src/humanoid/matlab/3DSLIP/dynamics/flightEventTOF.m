function [value isterminal direction] = flightEventTOF(t,state,dynParams)
        value = state(6);
        isterminal = 1;	direction = -1;
    end