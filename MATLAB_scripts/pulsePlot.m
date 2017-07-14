function [  ] = pulsePlot(Apulse,Bpulse,si,sq,n,fig)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
figure(fig)
plot(Apulse(n,:))
hold on
plot(Bpulse(n,:))
plot(si)
plot(sq)
hold off

end

