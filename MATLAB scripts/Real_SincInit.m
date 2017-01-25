function [ OS_table ] = Real_SincInit( bw, spb, ratio)
% OS_table = Real_SincInit(bw,cbw,spb,ratio)
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
for j = 0:len-1
    i = j/ratio - spb/2;
    
    if (i ~= 0)
        OS_table(j+STU) = SCALAR * sin(eta*i) / (eta*i);
    else
        OS_table(j+STU) = SCALAR;
    end
end

end

