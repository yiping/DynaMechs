function [ pcross ] = crossm( p )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

pcross = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
end

