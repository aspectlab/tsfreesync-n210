function [ output_args ] = plotPulses(si, Apulse, Bpulse)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
plot(si*max(max(Apulse),max(Bpulse)))
hold on
plot(Apulse)
plot(Bpulse)
hold off

end

