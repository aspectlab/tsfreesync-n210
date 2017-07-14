%%  AccuracyNew.m
%   This script finds the delay between a series of synchronized pulses and
%   writes 
% -0.0200   -0.0160   -0.0120   -0.0080   -0.0040         0    0.0040    0.0080    0.0120    0.0160    0.0200
%   Created: JC
%   May 20 2017: retained I and Q components for use in delay estimator TMC
%   May 29 2017: added templates for use in arctan fine delay estimator TMC
%% Cleanup
clear
clc

%% Initialize variables
DUR         = 30;           % length of recording in seconds
spb         = 1000;         % num Tx samples per buffer?
fstx        = 195e3;        % Tx sampling rate
fsrx        = 3e6;          % Rx sampling rate
tsrx        = 1/fsrx;
ratio       = fsrx/fstx;
PULSE_PER   = ratio*spb;    % period of pulse rate (in number of Rx samples, at rate fsrx)
                                % num Rx samples per buffer?
sinc_ratio  = 10000;
fnyeTx      = 0.5*fstx;     % nyquist for Tx(M/S) nodes
cbw         = 0.5*fnyeTx;   % sinc frequency shift
bw          = 0.9*cbw;      % sinc bw
noDelay     = 0.0;          % explicit 0 time shift
ampl        = 1;
dataLen     = DUR*fsrx;     % number of samples in each received file, assumed to be a multiple of PULSE_PER
start       = 1;            % How many buffers from the beginning of file to start analysis at
plot_scale  = 100;          % Scales the window of the pulse train plot

%% intermediate variables
newDataLen = dataLen-2*PULSE_PER;  % new length after trimming below
pulseCount = newDataLen / PULSE_PER;

% File Path
fold = '~/russula_mount/N210_dev/DATDump/';

%% Load each data file
fid=fopen(strcat(fold,'RX2-A.dat'),'r');
ChanA= fread(fid,Inf,'int16')';
fclose(fid);

fid=fopen(strcat(fold,'RX2-B.dat'),'r');
ChanB= fread(fid,Inf,'int16')';
fclose(fid);

%% extract I channel and ignore Q
ChanA=ChanA(1:2:end);
ChanB=ChanB(1:2:end);
num_samps = length(ChanA);

%% trim one off the beginning and end of each received vector so pulses are roughly in the middle of a block of length PULSE_PER
thresh = (9/10)*max(ChanA);
val = 0; inc = 0;
% get first peak
while val < thresh
    inc = inc + 1;
    val = ChanA(inc);
end

peak1 = inc;
inc = inc + spb;
val = 0;
while val < thresh
    inc = inc + 1;
    val = ChanA(inc);
end
peak2 = inc;
frame_width = peak2-peak1;
sigstart = peak1 + floor(frame_width/2);
figure(69)
plot(ChanA(1:sigstart + 2*frame_width))
hold on
plot(ChanB(1:sigstart + 2*frame_width))
scatter(peak1,thresh,'r','*')
scatter(peak2,thresh,'r','*')
for n = 1:800
    scatter(sigstart,n,'.')
end
hold off
% [~,sigstart]=max(abs(ChanA(1:PULSE_PER)));
% sigstart=sigstart+PULSE_PER/2+spb*(start-1);
ChanA=ChanA(sigstart:sigstart+newDataLen-1);
ChanB=ChanB(sigstart:sigstart+newDataLen-1);
pulse_count = floor(num_samps/frame_width);
R1dif = zeros(pulse_count,1);

%% Initialize oversampled sinc pulse
% Real_SincInit.m generates an oversampled pulse that does not include an 
% imaginary component.
OS_Sinc = Real_SincInit(bw, spb, sinc_ratio);

% Generate Undersampled unshifted pulse
template = SincGen(OS_Sinc,ampl, PULSE_PER, noDelay);
t = (0:(PULSE_PER-1))*tsrx;
si    = (template.*cos(2*pi*cbw*t));
sq    = (template.*sin(2*pi*cbw*t));

%% Find each pulse peak in each RX file
for n = 1:length(ChanA)/PULSE_PER
    
    % compute cross-correlation between the two channels (A and B)
    Apulse  = ChanA((1:PULSE_PER)+(n-1)*PULSE_PER);
    Bpulse  = ChanB((1:PULSE_PER)+(n-1)*PULSE_PER);
%     corr1 = abs(xcorr(Apulse,Bpulse));
    ziA     = abs(xcorr(Apulse,si));
    zqA     = abs(xcorr(Apulse,sq));
    ziB     = abs(xcorr(Bpulse,si));
    zqB     = abs(xcorr(Bpulse,sq));
    
    [~,posA] = max(abs(ziA));
    [~,posB] = max(abs(ziB));
    
    
    
    % compute coarse offsets (at precision of receive sample period)
%     [~,coarse_offset1]=max(corr1);
%     coarse_offset1=coarse_offset1-PULSE_PER;
%     cOff = coarse_offset1;
    
    coarse_delayA = posA-PULSE_PER;
    coarse_delayB = posB-PULSE_PER;
    
    fine_delayA = atan2(zqA(posA),ziA(posA))/(pi*cbw);
    fine_delayB = atan2(zqB(posB),ziB(posB))/(pi*cbw);
    
    tot_delayA = coarse_delayA + fine_delayA;
    tot_delayB = coarse_delayB + fine_delayB;

%     retained code below
%     zi = abs(xcorr(iChanA((1:PULSE_PER)+(n-1)*PULSE_PER),cChanB((1:PULSE_PER)+(n-1)*PULSE_PER)));
%     zq = abs(xcorr(qChanA((1:PULSE_PER)+(n-1)*PULSE_PER),cChanB((1:PULSE_PER)+(n-1)*PULSE_PER)));
%     find_offset=atan2(zq(cOff),zi(cOff))/(pi*cbw);

    % use quadratic interpolation to compute fine offset
%     y=corr1(coarse_offset1+PULSE_PER-1:coarse_offset1+PULSE_PER+1);
%     fine_offset1=(y(1)-y(3))/2/(y(1)-2*y(2)+y(3));
%     end retained code
    
    % store offsets
    report = coarse_delayA - coarse_delayB;
    R1dif(n) = coarse_delayA - coarse_delayB;
end


%% Calculate Statistics (in units of actual time, not in fractional samples)
R1MeanOff = mean(R1dif)*(1/fsrx);
R1SD =      std(R1dif)*(1/fsrx);

%% Print Stats
fprintf('St. Dev:\t%1.3d ns\nMean:\t%1.3d ns\n\n',R1SD*1e9,R1MeanOff*1e9);

%% Plot Stats
figure(423)
plot(R1dif./ratio,'*')
xlabel('time (in SPB''s)')
ylabel('measured offset (in fractional Tx samples)')
grid on

figure(3)  % check to make sure we're not half-way out of phase
plot(abs(ChanA(1:round(length(ChanA)/plot_scale))))
hold on
plot(abs(ChanB(1:round(length(ChanA)/plot_scale))),'r')
hold off

% fid = fopen('./data.txt','a');
% fprintf(fid,'Folder: %s\n\tMean Offset:\t%d samples\t%d ns\n\tSt. Dev:\t\t%d samples\t%d ns\n\n',...
%                         fold,R1MeanOff*fstx,R1MeanOff*1e9,R1SD*fstx,R1SD*1e9);
% fclose(fid);

% data = [fold,',',num2str(R1MeanOff*1e9),',',num2str(R1SD*1e9),'\n'];
% dlmwrite('data.csv',data,'-append');

% fid = fopen('data0.csv','a');
% fprintf(fid,data);
% fclose(fid);
