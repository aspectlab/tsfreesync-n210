%%  Accuracy.m
%   This script finds the delay between a series of synchronized pulses and
%   writes 
% -0.0200   -0.0160   -0.0120   -0.0080   -0.0040         0    0.0040    0.0080    0.0120    0.0160    0.0200

%% Initialize variables
clear


spb         = 1000;
mins        = 0;
secs        = 30;
DUR         = 60*mins + secs;            % length of recording in seconds
fsTx        = 250e3;      % Tx sampling rate
fsRx        = 2.5e6;        % Rx sampling rate
TsRx        = 1/fsRx;
ratio       = fsRx/fsTx;
PULSE_PER   = ratio*spb;   % period of pulse rate (in number of Rx samples, at rate fsrx)
dataLen     = DUR*fsRx;     % number of samples in each received file, assumed to be a multiple of PULSE_PER
start       = 1;          % How many buffers from the beginning of file to start analysis at
plot_scale  = 100;  % Scales the window of the pulse train plot

% Sinc variables
fNyqTx      = 0.5*fsTx;     % nyquist for Tx(M/S) nodes

W0          = (1/2)*2*pi*fNyqTx;
w0          = W0 * TsRx;

%% intermediate variables
newDataLen  = dataLen-2*PULSE_PER;  % new length after trimming below
pulse_count = newDataLen / PULSE_PER;


%% Load each data file
% File Path
% fold = '~/russula_mount/N210_dev/DATDump/RealAndImagDelayedJitter/';

fold = '~/russula_mount/N210_dev/MATLAB_scripts/previous_dats/7_11_2017/debug2/';
fid         = fopen(strcat(fold,'RX2-A.dat'),'r');
ChanA       = fread(fid,Inf,'int16')';
fclose(fid);

fid         =fopen(strcat(fold,'RX2-B.dat'),'r');
ChanB       = fread(fid,Inf,'int16')';
fclose(fid);

%% extract I and ignore Q components, Q should be all zeros
ChanA       =ChanA(1:2:end);
ChanB       =ChanB(1:2:end);
ampl        =max(real([ChanA,ChanB]));

%% trim one off the beginning and end of each received vector so pulses are roughly in the middle of a block of length PULSE_PER
[~,sigstart]   =max(abs(ChanA(1:PULSE_PER)));
sigstart       =sigstart+PULSE_PER/2+spb*(start-1)+1; %spb is the wrong offset, fine if start = 1
ChanA       =ChanA(sigstart:sigstart+newDataLen-1);
ChanB       =ChanB(sigstart:sigstart+newDataLen-1);

Rldif0      = zeros(length(ChanA)/PULSE_PER,1);
Rldif1      = zeros(length(ChanA)/PULSE_PER,1);
Csdif0      = zeros(length(ChanA)/PULSE_PER,1);
Csdif1      = zeros(length(ChanA)/PULSE_PER,1);

%% Initialize oversampled sinc pulse
% SincInit.m generates an oversampled sinc with real and imaginary parts
cbw         = 0.50;
sbw         = 0.45;
sinc_ratio  = 10e3;
noDelay     = 0.0;

OS_Sinc = SincInit(sbw,cbw,spb,sinc_ratio);

% Generate Undersampled unshifted pulse
template    = SincGen(OS_Sinc,ampl,PULSE_PER,noDelay);
si          = real(template);
sq          = imag(template);

%% Find each pulse peak in each RX file
Apulse = zeros(pulse_count,PULSE_PER);
Bpulse = zeros(pulse_count,PULSE_PER);
for n = 1:pulse_count
    Apulse(n,:)= ChanA((n-1)*PULSE_PER+1:(n)*PULSE_PER);
    Bpulse(n,:)= ChanB((n-1)*PULSE_PER+1:(n)*PULSE_PER);
end


disp('hard mode engaged')
parfor n = 1:pulse_count
    
    % compute cross-correlation between the two channels (A and B)

    ziA     = xcorr(Apulse(n,:),si);
    zqA     = xcorr(Apulse(n,:),sq);
    ziB     = xcorr(Bpulse(n,:),si);
    zqB     = xcorr(Bpulse(n,:),sq);
    
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
    coarse_delay = coarse_delayA - coarse_delayB;

    
    thetaA = atan2(zqA(posA),ziA(posA));
    thetaB = atan2(zqB(posB),ziB(posB));
    fine_delay = (thetaA - thetaB)/(w0);
    
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
R1MeanOff = mean(Rldif1)*(1/fsRx);
R1SD =      std(Rldif1)*(1/fsRx);

%% Print Stats
fprintf('St. Dev:\t%1.3d ns\nMean:\t%1.3d ns\n\n',R1SD*1e9,R1MeanOff*1e9);

%% Plot Stats
figure(437)
hold off
plot(Rldif1/ratio,'b.')
hold on
% plot(Rldif0/ratio,'r.')
% plot(Csdif0/ratio,'r-')
% plot(Csdif1/ratio,'c-')
title('sliding slave node (both)')
xlabel('time (in Tx buffers)')
ylabel('measured offset (in fractional Tx samples)')
grid on
hold off

% figure(3)  % check to make sure we're not half-way out of phase
% plot(abs(ChanA(1:round(length(ChanA)/plot_scale))))
% hold on
% plot(abs(ChanB(1:round(length(ChanA)/plot_scale))),'r')
% hold off

figure(3)
plot(Apulse(69,:))
hold on
plot(Bpulse(69,:))
plot(si)
plot(sq)
hold off

% fid = fopen('./data.txt','a');
% fprintf(fid,'Folder: %s\n\tMean Offset:\t%d samples\t%d ns\n\tSt. Dev:\t\t%d samples\t%d ns\n\n',...
%                         fold,R1MeanOff*fstx,R1MeanOff*1e9,R1SD*fstx,R1SD*1e9);
% fclose(fid);
% 
% data = [fold,',',num2str(R1MeanOff*1e9),',',num2str(R1SD*1e9),'\n'];
% dlmwrite('data.csv',data,'-append');
% 
% fid = fopen('data0.csv','a');
% fprintf(fid,data);
% fclose(fid);
