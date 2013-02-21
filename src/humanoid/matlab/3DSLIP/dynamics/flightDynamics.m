function dState = flightDynamics(t,state,dynParams)
  dState = 0*state;
  dState(1:3) = state(4:6);
  dState(6)   = -dynParams.g;
end