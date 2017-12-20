%%% auto_accuracy.m
% computes delay statistics from a series of .dat files

num_debugs = 100;
date_vec = [9 20];


% 0 if running on cubensis, 1 if on russula
on_russula = 0;

DURATION = 300;
spb     = 1e3;
fsRx = 2.50e6;
fsTx = 250e3;
samp_ratio = fsRx/fsTx;
spp = spb*samp_ratio;

cbw = .5;
sbw = .45;

%verbos, but carries a lot of meaning
fs_nyq = fsTx/2;
W0 = cbw * 2 * pi * fs_nyq;
w0 = W0 / fsRx;
%%%
sinc_ratio  = 10e3;
noDelay     = 0.0;
ampl        = 1024;

OS_Sinc = SincInit(sbw,cbw,spb,sinc_ratio);

% Generate Undersampled unshifted pulse
template    = SincGen(OS_Sinc,ampl,spp,noDelay);
si          = real(template);
sq          = imag(template);

stats = zeros(num_debugs,2);
pulse_count = zeros(1,num_debugs);
if on_russula
    N210 = '~/N210';
else
    N210 = '~/russula_mount/N210_dev';
end
form_spec   = '/Dat_Dumps/rx_dumps/%i_%i_2017/';
debug_dir = sprintf(form_spec,date_vec);

log = fopen(strcat(N210,debug_dir,'log.txt'),'a');

date_str = datestr(datetime('now'));
fprintf(log,'initialization complete at:');
fprintf(log,date_str);
fprintf(log,'\n\n');
fprintf('initialization complete\n');
for debug_num = 1:num_debugs
% for debug_num = 79
    date_str = datestr(datetime('now'));
    formspec = strcat('debug%i: ',date_str,'\n');
    fprintf(log,formspec,debug_num);
    fprintf(formspec,debug_num);
 

%     folder = strcat(pwd,sprintf(form_spec,dir));
    debug_fold = strcat('/debug',num2str(debug_num));
    folder = fullfile(N210,debug_dir,debug_fold);
    
    fid         = fopen(strcat(folder,'/RX2-A.dat'),'r');
    ChanA       = fread(fid,Inf,'int16')';
    ChanA       = ChanA(1:2:end);
    fclose(fid);
    disp('read file:1')
    
    fid         = fopen(strcat(folder,'/RX2-B.dat'),'r');
    ChanB       = fread(fid,Inf,'int16')';
    ChanB       = ChanB(1:2:end);
    fclose(fid);
    disp('read file:2')
  
    %% trim one off the beginning and end of each received vector so pulses are roughly in the middle of a block of length PULSE_PER
    dataLen = length(ChanA);
    newDataLen  = dataLen-2*spp;  % new length after trimming below
    pulse_count =floor(newDataLen / spp);
    pulse_count_mem(debug_num) = pulse_count;
    
    [~,sigstart]   =max(abs(ChanA(1:spp)));
    sigstart       =sigstart+spp/2+1; 
    ChanA       =ChanA(sigstart:end);
    ChanB       =ChanB(sigstart:end);
 
    Rldif      = zeros(pulse_count,1);
    

    
    %% Find each pulse peak in each RX file
    Apulse = zeros(pulse_count,spp);
    Bpulse = zeros(pulse_count,spp);
    disp('engaging chopper')
    for n = 1:pulse_count
        Apulse(n,:)= ChanA((n-1)*spp+1:(n)*spp);
        Bpulse(n,:)= ChanB((n-1)*spp+1:(n)*spp);
    end
    
    n_tile = round(pulse_count/4);
    
    
    disp('hard mode engaged')
    parfor n = 1:pulse_count
        if mod(n,n_tile) == 0 %&& (n > pulse_count/2)
            fprintf('~%d%% complete\n',round(100*n/pulse_count));
        end
        
%         ziA     = conv(Apulse(n,:),fliplr(si));
%         zqA     = conv(Apulse(n,:),fliplr(sq));
%         ziB     = conv(Bpulse(n,:),fliplr(si));
%         zqB     = conv(Bpulse(n,:),fliplr(sq));

%         [~,posA] = max(ziA.^2+zqA.^2);
%         [~,posB] = max(ziB.^2+zqB.^2);

%         thetaA = atan2(zqA(posA),ziA(posA));
%         thetaB = atan2(zqB(posB),ziB(posB));

        zA = conv(Apulse(n,:),fliplr(template));
        zB = conv(Bpulse(n,:),fliplr(template));
        
        
        % compute coarse offsets (at precision of receive sample period)
        [~,posA] = max(real(zA).^2+imag(zA).^2);
        [~,posB] = max(real(zB).^2+imag(zB).^2);

        
        coarse_delayA = posA-spp;
        coarse_delayB = posB-spp;

        
        thetaA = atan2(imag(zA(posA)),real(zA(posA)));
        thetaB = atan2(imag(zB(posB)),real(zB(posB)));

        coarse_delay = coarse_delayA - coarse_delayB;
        fine_delay = (thetaA - thetaB)/(w0);
        
        % store offsets
        Rldif(n) = fine_delay + coarse_delay;
    end
    [mu, sig] = normfit(Rldif);
    mu = mu * 1e9/fsRx;
    sig = sig * 1e9/fsRx;
    fprintf(log,'debug%i complete, mu:%d sig:%d\n',[debug_num,mu,sig]);
    filename = sprintf('/auto_accuracy_debug%i.mat',debug_num);
    save(strcat(folder,filename),'mu','sig','Rldif')
    
    
    %% Calculate Statistics (in units of actual time, not in fractional samples)
    stats(debug_num,1) = mean(Rldif)*(1/fsRx);
    stats(debug_num,2) = std(Rldif)*(1/fsRx);
end
date_str = datestr(datetime('now'));
fprintf(log,'finished at:');
fprintf(log,date_str);
fprintf(log,'\n\n\n');
subplot 211
plot(stats(:,2))
subplot 212
histogram(stats(:,2))
filename = sprintf('stats_%i_%i',date_vec);
save(strcat(N210,debug_dir,filename),'stats')
fclose(log);