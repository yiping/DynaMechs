function [ Ibar ] = IcmbarToIbar(m, Icmbar, c_g )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Ibar = Icmbar + m * cross(c_g)* (cross(c_g)');
end

