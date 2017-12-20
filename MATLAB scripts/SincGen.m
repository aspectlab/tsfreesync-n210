function [ table ] = SincGen(template, ampl, spb, delay )
% table = SincGen(template,ampl,spb,delay)
% This function takes the oversampled pulse from SincInit.m and
% undersamples it with a shift
% This function requires SincInit.m to be run first
% Outputs :
%   table, the undersampled shifted pulse
% Inputs :
%   template, the oversampled pulse from SincInit.m
%   spb, the length of the undersampled pulse
%   delay, number of samples to delay the undersampled pulse
% Joseph Canfield, January 3, 2017

global SCALAR;
global STU;

ratio = length(template)/spb;
scale = ampl/SCALAR;

if (delay < 0.0)
    delay = delay + spb;
end

    % keep delay below spb
delay = rem(delay,spb);

% Scale the delay to a number of samples in the oversampled pulse
shift = round(delay*ratio);

n = 0;
l = 0;
table = zeros(1,spb);

% Length of the oversampled pulse
OS_len = spb*ratio;

% Generate first part of delayed pulse
for l = shift:ratio:OS_len-1
    table(n+STU) = scale * template(floor(l+STU));
    n = n + 1;
end

% Save the amount of samples from the end for calculating beginning
remain = OS_len-l;

% Generate second part of delayed pulse
for l = ratio-remain:ratio:shift-ratio-1
    table(n+STU) = -scale * template(floor(l+STU));
    n = n + 1;
end

end

