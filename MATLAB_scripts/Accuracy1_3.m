%%  Accuracy.m
%   This script finds the delay between a series of synchronized pulses and
%   writes 
% -0.0200   -0.0160   -0.0120   -0.0080   -0.0040         0    0.0040    0.0080    0.0120    0.0160    0.0200

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
plot_scale  = 100;  % Scales the window of the pulse train plot

% Sinc variables
sinc_ratio  = 10000;
fNyqTx      = 0.5*fstx;     % nyquist for Tx(M/S) nodes
cbw         = 0.5*fNyqTx;   % sinc frequency shift
sinc_bw     = .45;          % normalized sinc bw
noDelay     = 0.0;          % explicit 0 delay
ampl        = 1;

W0          = 2*pi*cbw;
w0          = W0 * tsrx;

%% intermediate variables
newDataLen = dataLen-2*PULSE_PER;  % new length after trimming below
pulse_count = newDataLen / PULSE_PER;

% File Path
% fold = '~/russula_mount/N210_dev/DATDump/RealAndImagDelayedJitter/';
fold = '~/russula_mount/N210_dev/MATLAB scripts/previous dats/rate_est_0p999/';
%% Load each data file
fid=fopen(strcat(fold,'RX2-A.dat'),'r');
ChanA= fread(fid,Inf,'int16')';
fclose(fid);

fid=fopen(strcat(fold,'RX2-B.dat'),'r');
ChanB= fread(fid,Inf,'int16')';
fclose(fid);

%% extract I and ignore Q components, Q should be all zeros
ChanA=ChanA(1:2:end);
ChanB=ChanB(1:2:end);

%% trim one off the beginning and end of each received vector so pulses are roughly in the middle of a block of length PULSE_PER
[~,sigstart]=max(abs(ChanA(1:PULSE_PER)));
sigstart=sigstart+PULSE_PER/2+spb*(start-1); %spb is the wrong offset, fine if start = 1
ChanA=ChanA(sigstart:sigstart+newDataLen-1);
ChanB=ChanB(sigstart:sigstart+newDataLen-1);

Rldif0 = zeros(length(ChanA)/PULSE_PER,1);
Rldif1 = zeros(length(ChanA)/PULSE_PER,1);
Csdif0 = zeros(length(ChanA)/PULSE_PER,1);
Csdif1 = zeros(length(ChanA)/PULSE_PER,1);

%% Initialize oversampled sinc pulse
% Real_SincInit.m generates an oversampled pulse that does not include an 
% imaginary component.
OS_Sinc = Real_SincInit(sinc_bw, spb, sinc_ratio);

% Generate Undersampled unshifted pulse
template = SincGen(OS_Sinc,ampl, PULSE_PER, noDelay);
t     = (1-PULSE_PER/2:PULSE_PER/2);
si    = (template.*cos(w0*t));
sq    = (template.*sin(w0*t));

%% Find each pulse peak in each RX file
Apulse = zeros(pulse_count,PULSE_PER);
Bpulse = zeros(pulse_count,PULSE_PER);
for n = 1:pulse_count
    Apulse(n,:)= ChanA((n-1)*PULSE_PER+1:(n)*PULSE_PER);
    Bpulse(n,:)= ChanB((n-1)*PULSE_PER+1:(n)*PULSE_PER);
end



parfor n = 1:pulse_count
    
    % compute cross-correlation between the two channels (A and B)

    ziA     = abs(xcorr(Apulse(n,:),si));
    zqA     = abs(xcorr(Apulse(n,:),sq));
    ziB     = abs(xcorr(Bpulse(n,:),si));
    zqB     = abs(xcorr(Bpulse(n,:),sq));
    
    % compute coarse offsets (at precision of receive sample period)
    [~,posA] = max(ziA.^2+zqA.^2);
    [~,posB] = max(ziB.^2+zqB.^2);
    
%     figure(42)
%     plot(Apulse(n,:))
%     hold on
%     plot(Bpulse(n,:))
%     plot(max(Apulse(n,:))*si)
%     hold off
%     n
    
    coarse_delayA = posA-PULSE_PER;
    coarse_delayB = posB-PULSE_PER;
    coarse_delay = coarse_delayB - coarse_delayA;
    
    thetaA = atan2(zqA(posA),ziA(posA));
    thetaB = atan2(zqB(posB),ziB(posB));
    fine_delay = (thetaB - thetaA)/(w0);
    
    corr1 = abs(xcorr(Apulse(n,:),Bpulse(n,:)));
    [~,coarse_offset1]=max(corr1);
    coarse_offset1=coarse_offset1-PULSE_PER;
    
    y=corr1(coarse_offset1+PULSE_PER-1:coarse_offset1+PULSE_PER+1);
    fine_offset1=(y(1)-y(3))/2/(y(1)-2*y(2)+y(3));
    
%     mid = ceil(((posA+posB)/2)/2);
%     rng = abs(posA-posB);
%     lo  = mid-rng;
%     hi  = mid+rng;
    
%     figure(42)
%     plot(Apulse)
%     hold on
%     plot(Bpulse)
%     plot(template*max(Apulse))
%     hold off
%     axis([lo hi 0 1800])
    
    % store offsets
    Rldif0(n) = coarse_offset1 + fine_offset1;
    Rldif1(n) = coarse_delay + fine_delay;
    Csdif1(n) = coarse_delay;
    Csdif0(n) = coarse_offset1;
end


%% Calculate Statistics (in units of actual time, not in fractional samples)
R1MeanOff = mean(Rldif1)*(1/fsrx);
R1SD =      std(Rldif1)*(1/fsrx);

%% Print Stats
fprintf('St. Dev:\t%1.3d ns\nMean:\t%1.3d ns\n\n',R1SD*1e9,R1MeanOff*1e9);

%% Plot Stats
figure(426)
hold off
plot(Rldif1./ratio,'b.')
hold on
plot(Rldif0./ratio,'r.')
plot(Csdif0./ratio,'r-')
plot(Csdif1./ratio,'b-')
title('arctan estimator')
xlabel('time (in SPB''s)')
ylabel('measured offset (in fractional Tx samples)')
grid on
hold off

figure(3)  % check to make sure we're not half-way out of phase
plot(abs(ChanA(1:round(length(ChanA)/plot_scale))))
hold on
plot(abs(ChanB(1:round(length(ChanA)/plot_scale))),'r')
hold off

fid = fopen('./data.txt','a');
fprintf(fid,'Folder: %s\n\tMean Offset:\t%d samples\t%d ns\n\tSt. Dev:\t\t%d samples\t%d ns\n\n',...
                        fold,R1MeanOff*fstx,R1MeanOff*1e9,R1SD*fstx,R1SD*1e9);
fclose(fid);

data = [fold,',',num2str(R1MeanOff*1e9),',',num2str(R1SD*1e9),'\n'];
dlmwrite('data.csv',data,'-append');

fid = fopen('data0.csv','a');
fprintf(fid,data);
fclose(fid);
