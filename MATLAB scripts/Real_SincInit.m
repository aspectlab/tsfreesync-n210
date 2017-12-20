function [ OS_table ] = Real_SincInit( bw, spb, ratio)
% OS_table = Real_SincInit(bw,spb,ratio)
% This script generates an oversampled Sinc pulse based on the precision
% parameter, where precision is the sample period in seconds.
% This script does not include an imaginary component for the sinc pulse
% nor does it modulate the pulse.
% This script is designed to be used by BandWidthTests.m
% Outputs :
%   OS_table, the oversampled sinc pulse
% Inputs :
%   bw, normalized bandwidth of the pulse
%   spb, number of samples per buffer in the downsampled version(usually
%   1000)
%   ratio, ratio samples between oversampled and downsampled pulses
% Joseph Canfield, January 3, 2017

global STU;            % Stupid constant because MATLAB doesn't like 0
STU = 1;
global SCALAR;
SCALAR = 32767;   

len = spb*ratio;   % length of the oversampled pulse

eta = bw*pi;    % eta controls BW of sinc pulse

% Init
OS_table = zeros(1,floor(len));

% Calculate Pulse
for n = 0:len-1
    l = n/ratio - spb/2; % (-spb/2:(spb-1)/2)/ratio
    
    if (l ~= 0)
        OS_table(n+STU) = SCALAR * sin(eta*l) / (eta*l);
    else
        OS_table(n+STU) = SCALAR;
    end
end

end

