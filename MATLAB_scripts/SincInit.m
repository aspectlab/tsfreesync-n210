function [ OS_table ] = SincInit( sbw, cbw, spb, ratio)
% OS_table = SincInit(bw,cbw,spb,ratio)
% This script generates an oversampled Sinc pulse
% Outputs :
%   OS_table, the oversampled sinc pulse
% Inputs :
%   bw, normalized bandwidth of the pulse
%   cbw, normalized carrier bandwidth of the pulse
%   spb, number of samples per buffer in the downsampled version(usually
%   1000)
%   ratio, ratio samples between oversampled and downsampled pulses
% Joseph Canfield, July 27, 2016

global STU;         % Stupid constant because MATLAB doesn't like 0
STU = 1;
global SCALAR;      % Largest possible value for signed 16 bit integers
SCALAR = 32767;

len = spb*ratio;   % length of the oversampled pulse

w0  = cbw*pi;   % w0 controls frequency shift
eta = sbw*pi;    % eta controls BW of sinc pulse
modDelay = 0;   % num samples to delay modulation functions

% Init
OS_table = zeros(1,floor(len));

% Calculate Pulse
% l > samples in oversampled domain
% n > samples at the receiver (all nodes are receivers)
for k = 0:len-1
    l = k/ratio - spb/2;
    
    if (l ~= 0)
        OS_table(k+STU) = complex(...
            SCALAR * sin(eta*l) / (eta*l) * cos(w0*(l+modDelay)),...
            SCALAR * sin(eta*l) / (eta*l) * sin(w0*(l+modDelay)));
    else
        OS_table(k+STU) = complex(SCALAR, 0.0);
    end
end

end

