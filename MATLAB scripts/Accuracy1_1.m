%%  AccuracyNew.m
%   This script finds the delay between a series of synchronized pulses and
%   writes 
% -0.0200   -0.0160   -0.0120   -0.0080   -0.0040         0    0.0040    0.0080    0.0120    0.0160    0.0200
%   Created: JC
%   May 20 2017: retained I and Q components for use in delay estimator TMC
%   May 29 2017: added templates for use in arctan fine delay estimator TMC
%% Initialize variables

clear
clc

spb         = 1000;         % num Tx samples per buffer?
DUR         = 30;           % length of recording in seconds
fstx        = 100e3;        % Tx sampling rate
fsrx        = 3e6;              % Rx sampling rate
tsrx        = 1/fsrx;
ratio       = fsrx/fstx;
sinc_ratio  = 10000;
fnyeTx      = 0.5*fstx;     % nyquist for Tx(M/S) nodes
cbw         = 0.5*fnyeTx;   % sinc frequency shift
bw          = 0.9*cbw;      % sinc bw
wo          = cbw*pi*tsrx;
PULSE_PER   = ratio*spb;    % period of pulse rate (in number of Rx samples, at rate fsrx)
                                % num Rx samples per buffer?
noDelay     = 0.0;          % explicit 0 time shift
ampl        = 1;
dataLen     = DUR*fsrx;     % number of samples in each received file, assumed to be a multiple of PULSE_PER
start       = 1;            % How many buffers from the beginning of file to start analysis at
plot_scale  = 100;          % Scales the window of the pulse train plot

%% intermediate variables
newDataLen = dataLen-2*PULSE_PER;  % new length after trimming below
pulseCount = newDataLen / PULSE_PER;

% File Path
%fold = './0.4b-0.5c/0.0000/';
fold = '~/russula_mount/N210_dev/DATDump/';

%% Load each data file
fid=fopen(strcat(fold,'RX2-A.dat'),'r');
ChanA= fread(fid,Inf,'int16')';
fclose(fid);

fid=fopen(strcat(fold,'RX2-B.dat'),'r');
ChanB= fread(fid,Inf,'int16')';
fclose(fid);

%% extract I and Q and combine into complex vector
iChanA=ChanA(1:2:end);
qChanA=ChanA(2:2:end);
cChanA=iChanA+1j*qChanA;

iChanB=ChanB(1:2:end);
qChanB=ChanB(2:2:end);
cChanB=iChanB+1j*qChanB;
% retained code below
% cChanB=ChanB(1:2:end)+1j*ChanB(2:2:end);
% cChanB=ChanB(1:2:end)+1j*ChanB(2:2:end);
% end retained code

%% trim one off the beginning and end of each received vector so pulses are roughly in the middle of a block of length PULSE_PER
[~,sigstart]=max(abs(cChanA(1:PULSE_PER)));
sigstart=sigstart+PULSE_PER/2+spb*(start-1);
% attempted to trim i and q similarly to c
iChanA=iChanA(sigstart:sigstart+newDataLen-1);
qChanA=qChanA(sigstart:sigstart+newDataLen-1);
cChanA=cChanA(sigstart:sigstart+newDataLen-1);
iChanB=iChanB(sigstart:sigstart+newDataLen-1);
qChanB=qChanB(sigstart:sigstart+newDataLen-1);
cChanB=cChanB(sigstart:sigstart+newDataLen-1);

offset1=zeros(length(cChanA)/PULSE_PER,1);
offset2=zeros(length(cChanA)/PULSE_PER,1);
R1dif = zeros(length(cChanA)/PULSE_PER,1);

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
for n = 1:length(cChanA)/PULSE_PER
    
    % compute cross-correlation between the two channels (A and B)
    Apulsei  = iChanA((1:PULSE_PER)+(n-1)*PULSE_PER);
    Bpulsei  = iChanB((1:PULSE_PER)+(n-1)*PULSE_PER);
    Bpulseq  = qChanB((1:PULSE_PER)+(n-1)*PULSE_PER);
%     corr1 = abs(xcorr(Apulse,Bpulse));
    zi     = abs(xcorr(Apulsei,Bpulsei));
    zq     = abs(xcorr(Apulsei,Bpulseq));
%     zi = abs(xcorr(iChanA((1:PULSE_PER)+(n-1)*PULSE_PER),iChanB((1:PULSE_PER)+(n-1)*PULSE_PER)));
%     zq = abs(xcorr(iChanA((1:PULSE_PER)+(n-1)*PULSE_PER),qChanB((1:PULSE_PER)+(n-1)*PULSE_PER)));
    
    [~,pos] = max(abs(zi));
    
    
    
    % compute coarse offsets (at precision of receive sample period)
%     [~,coarse_offset1]=max(corr1);
%     coarse_offset1=coarse_offset1-PULSE_PER;
%     cOff = coarse_offset1;
    
    coarse_delay = pos-PULSE_PER;
   

%     retained code below

%     find_offset=atan2(zq(cOff),zi(cOff))/(pi*cbw);

    fine_delay = atan2(zq(pos),zi(pos))/(cbw*pi*tsrx);
    
    tot_delay = coarse_delay + fine_delay;

    % use quadratic interpolation to compute fine offset
%     y=corr1(coarse_offset1+PULSE_PER-1:coarse_offset1+PULSE_PER+1);
%     fine_offset1=(y(1)-y(3))/2/(y(1)-2*y(2)+y(3));
%     end retained code
    
    % store offsets
    report = fine_delay;
    R1dif(n) = fine_delay;
end


%% Calculate Statistics (in units of actual time, not in fractional samples)
R1MeanOff = mean(R1dif)*(1/fsrx);
R1SD =      std(R1dif)*(1/fsrx);

%% Print Stats
fprintf('St. Dev:\t%1.3d ns\nMean:\t%1.3d ns\n\n',R1SD*1e9,R1MeanOff*1e9);

%% Plot Stats
figure(422)
plot(R1dif./ratio,'*')
title('arctan estimator')
xlabel('time (in SPB''s)')
ylabel('measured offset (in fractional Tx samples)')
grid on

figure(3)  % check to make sure we're not half-way out of phase
plot(abs(cChanA(1:round(length(cChanA)/plot_scale))))
hold on
plot(abs(cChanB(1:round(length(cChanA)/plot_scale))),'r')
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
