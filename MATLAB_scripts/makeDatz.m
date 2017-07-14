%% Generates sample data for use with Accuracy


%% Initialize variables
spb         = 1000;
DUR         = 30;            % length of recording in seconds
fstx        = 250e3;      % Tx sampling rate
fsrx        = 5e6;        % Rx sampling rate
tsrx        = 1/fsrx;
ratio       = fsrx/fstx;
PULSE_PER   = ratio*spb;   % period of pulse rate (in number of Rx samples, at rate fsrx)
dataLen     = DUR*fsrx;     % number of samples in each received file, assumed to be a multiple of PULSE_PER
start       = 1;          % How many buffers from the beginning of file to start analysis at

%% Initialize oversampled sinc pulse
% Real_SincInit.m generates an oversampled pulse that does not include an 
% imaginary component.
OS_Sinc = Real_SincInit(sinc_bw, spb, sinc_ratio);

% Generate Undersampled unshifted pulse
template = SincGen(OS_Sinc,ampl, PULSE_PER, noDelay);
