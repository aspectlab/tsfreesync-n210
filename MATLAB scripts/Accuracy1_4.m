%%  Accuracy.m
%   This script finds the delay between a series of synchronized pulses and
%   writes 
% -0.0200   -0.0160   -0.0120   -0.0080   -0.0040         0    0.0040    0.0080    0.0120    0.0160    0.0200

%% Initialize variables
clear
% old file path
% date_vec = [9,20];
% debug_num = 79;
% dir_vec = [date_vec, debug_num];
% folder = sprintf('~/russula_mount/N210_dev/Dat_Dumps/rx_dumps/%i_%i_2017/debug%i',dir_vec);

% new file path
config = 1
fixed = 1;
if fixed
    file_str = 'fixed'
else
    file_str = 'final'
end
% folder = sprintf('~/russula_mount/N210_dev/Dat_Dumps/config/%s_config/config%i',file_str,config);
folder = sprintf('~/N210_dev/rxnode/build/');
%sprintf('~/russula_mount/N210_dev/MATLAB_scripts/previous_dats/11_30_2017');

spb         = 1000;
mins        = 1/2;
secs        = 0;
DUR         = 60*mins + secs;            % length of recording in seconds
fsTx        = 250e3;      % Tx sampling rate
fsRx        = 5e6;      % Rx sampling rate
TsRx        = 1/fsRx;
samp_ratio       = fsRx/fsTx;
spp         = samp_ratio*spb;   % period of pulse rate (in number of Rx samples, at rate fsrx)
dataLen     = DUR*fsRx;     % number of samples in each received file, assumed to be a multiple of PULSE_PER
start       = 1;          % How many buffers from the beginning of file to start analysis at
plot_scale  = 100;  % Scales the window of the pulse train plot

% Sinc variables
fNyqTx      = 0.5*fsTx;     % nyquist for Tx(M/S) nodes

W0          = (1/2)*2*pi*fNyqTx;
w0          = W0 * TsRx;

%% intermediate variables
newDataLen  = dataLen-2*spp;  % new length after trimming below
pulse_count = newDataLen / spp;


%% Load each data file
% File Path
% fold = '~/russula_mount/N210_dev/DATDump/RealAndImagDelayedJitter/';

% current directory should be MATLAB_scripts
% folder = strcat(pwd,'/previous_dats/9_12_2017/debug2/');


fid         = fopen(strcat(folder,'/RX2-A.dat'),'r');
ChanA       = fread(fid,Inf,'int16')';
ChanA       =ChanA(1:2:end);
fclose(fid);
disp('read file:1')

fid         =fopen(strcat(folder,'/RX2-B.dat'),'r');
ChanB       = fread(fid,Inf,'int16')';
ChanB       =ChanB(1:2:end);
fclose(fid);
disp('read file:2')
%% extract I and ignore Q components, Q should be all zeros




%% trim one off the beginning and end of each received vector so pulses are roughly in the middle of a block of length PULSE_PER
[~,sigstart]   =max(abs(ChanA(1:spp)));
sigstart       =sigstart+spp/2+1; %spb is the wrong offset, fine if start = 1
ChanA       =ChanA(sigstart:end);
ChanB       =ChanB(sigstart:end);

Rldif      = zeros(pulse_count,1);

%% Initialize oversampled sinc pulse
% SincInit.m generates an oversampled sinc with real and imaginary parts
cbw         = 0.50;
sbw         = 0.02;
sinc_ratio  = 10e3;
noDelay     = 0.0;
ampl        = 1024;

OS_Sinc = SincInit(sbw,cbw,spb,sinc_ratio);

% Generate Undersampled unshifted pulse
template    = SincGen(OS_Sinc,ampl,spp,noDelay);
si          = real(template);
sq          = imag(template);

%% Find each pulse peak in each RX file
Apulse = zeros(pulse_count,spp);
Bpulse = zeros(pulse_count,spp);
disp('engage chopper')
for n = 1:pulse_count
    Apulse(n,:)= ChanA((n-1)*spp+1:(n)*spp);
    Bpulse(n,:)= ChanB((n-1)*spp+1:(n)*spp);
end

n_tile = round(pulse_count/42);


disp('hard mode engaged')
parfor n = 1:pulse_count
    if (mod(n,n_tile) == 0)
        percent = round(100*n/pulse_count);
        fprintf('~%i%% complete\n',percent)
    end

    ziA     = conv(Apulse(n,:),fliplr(si));
    zqA     = conv(Apulse(n,:),fliplr(sq));
    ziB     = conv(Bpulse(n,:),fliplr(si));
    zqB     = conv(Bpulse(n,:),fliplr(sq));
    
    % compute coarse offsets (at precision of receive sample period)
    [~,posA] = max(ziA.^2+zqA.^2);
    [~,posB] = max(ziB.^2+zqB.^2);
    
    coarse_delayA = posA-spp;
    coarse_delayB = posB-spp;
    coarse_delay = coarse_delayA - coarse_delayB;
    
    thetaA = atan2(zqA(posA),ziA(posA));
    thetaB = atan2(zqB(posB),ziB(posB));
    fine_delay = (thetaA - thetaB)/(w0);
    
        
    % store offsets
    Rldif(n) = fine_delay + coarse_delay;
end


%% Calculate Statistics (in units of actual time, not in fractional samples)
R1MeanOff = mean(Rldif)*(1/fsRx);
R1SD =      std(Rldif)*(1/fsRx);
R1Drift = mean(diff(Rldif))*(1/fsRx);
stats = [R1MeanOff,R1SD,R1Drift];
%% Print Stats
fprintf('St. Dev\t      = %1.3d ns\nMean\t      = %1.3d ns\n\n',R1SD*1e9,R1MeanOff*1e9);

%% Plot Stats
figure(437)
hold off
plot(Rldif*(fsTx/fsTx),'b.-')
hold on
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
plot(Apulse(42,:))
hold on
plot(Bpulse(42,:))
plot(si)
plot(sq)
hold off

filename = sprintf('/accuracy_config%i.mat',config);
save(strcat(folder,filename),'Rldif','R1MeanOff','R1SD','R1Drift')


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
