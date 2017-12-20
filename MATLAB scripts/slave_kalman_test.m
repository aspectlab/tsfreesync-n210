%%% slave_kalman_test.m
% reads in information from a slave dump and calculates the optimal kalman
% gains for estimating the master's clock offset


clear

dir   = [9,11,1];
form_spec   = '/previous_dats/%i_%i_2017/debug%i';
folder      = strcat(pwd,sprintf(form_spec,dir));


%%
file = strcat(folder,'/README.txt');
fid = fopen(file,'r');
debug_cell=textscan(fid,'%s %f');
fclose(fid);
debug_names = debug_cell{1};
debug_vals = debug_cell{2};


% fread_buff = fread(fid,Inf,'char');
for i=1:size(debug_names,1)
    eval([debug_names{i} '=' num2str(debug_vals(i))]);
end


fid         = fopen(strcat(folder,'/rx.dat'),'r');
fread_buff  = fread(fid,Inf,'int16')';
rx_data      = fread_buff(1:2:end);
fclose(fid);

fid         = fopen(strcat(folder,'/tx.dat'),'r');
fread_buff  = fread(fid,Inf,'int16')';
tx_data     = fread_buff(1:2:end);
fclose(fid);

fid         = fopen(strcat(folder,'/pred_error.dat'),'r');
pred_err_dat= fread(fid,Inf,'double');

tx_data     = [zeros(1,(TXDELAY+1)*SPB) tx_data];
tx_data     = tx_data(1:end-(TXDELAY+1)*SPB);

%% Generate local copy of sinc pulse
ratio = 1;
delay = 0;
ampl  = 1024;
OS_Sinc = SincInit(BW,CBW,SPB,ratio);
template    = SincGen(OS_Sinc,ampl,SPB,delay);
si          = fix(real(template)); % <--- THIS IS THE ACTUAL SINC PULSE
sq          = fix(imag(template));
tx_thresh   = 20e3*ampl;

%% find transmitted sinc pulses on primary channel
tx_pulse_time = find(conv(fliplr(si),tx_data)>tx_thresh)-SPB+1;
% quick sanity check
if any(diff(tx_pulse_time)~=SPB*SYNC_PERIOD)
    error('Transmitted data is not as expected (i.e., pulses don''t occur with correct period. This could also happen if tx_threshold is set incorrectly, or you''re doing things this code wasn''t built to deal with.')
end

%% find received sinc pulses
num_samples = length(rx_data);
si_corr=conv(floor(fliplr(si)),rx_data); si_corr=si_corr(SPB:end);
sq_corr=conv(floor(fliplr(sq)),rx_data); sq_corr=sq_corr(SPB:end);
numXchanges = floor((length(si_corr)-tx_pulse_time(1)+1)/SPB/SYNC_PERIOD);
coarsetime = zeros(1,numXchanges);
finetime = zeros(1,numXchanges);
si_corr_max = zeros(1,numXchanges);    %
sq_corr_max = zeros(1,numXchanges);

for x=1:numXchanges
    startindex=(x-1)*SPB*SYNC_PERIOD+tx_pulse_time(1)-1;
    [~,peak_loc]=max(si_corr(1+startindex:startindex+SPB*SYNC_PERIOD));
    coarsetime(x)=startindex+peak_loc;
    finetime(x)=atan2(sq_corr(coarsetime(x)),si_corr(coarsetime(x)))/CBW/pi;
    si_corr_max(x) = si_corr(coarsetime(x));
    sq_corr_max(x) = sq_corr(coarsetime(x));
end
rx_pulse_time = coarsetime+FDEL_SCALE*finetime;
if any(tx_pulse_time(1:numXchanges)>rx_pulse_time(1:numXchanges)) || any(tx_pulse_time(2:numXchanges)<rx_pulse_time(1:numXchanges-1))
    error('Tx and Rx pulses are not all paired up (i.e. two of one type occurred before the other).')
end

%% compute clockoffsets times
tx_pulse_time = tx_pulse_time(1:numXchanges);
roundtrip_times = rx_pulse_time-tx_pulse_time+1;   % SHOULD THIS +1 be here?  need to think about this *****************************************




K0s = ceil(roundtrip_times/SPB)+2;                  % implicitly assume pulses are always found in "previous" buffer

K0_avg = round(mean(K0s));
if any(K0s ~= K0_avg)
    disp('inconsistant roundtrip times');
end
elapsed_time = K0s*SPB-roundtrip_times/2;
meas_master = mod(-elapsed_time - (TXDELAY + 1)*SPB,SPB*RATE_SEED);
meas_master(meas_master>SPB/2)=meas_master(meas_master>SPB/2)-SPB*RATE_SEED;

% skip = 2;
% z = zeros(1,skip);
% pred_master = [z pred_master];
% finetime = [z finetime];

%% analysis

k1_1min = [.65, 0.6932, 0.6863, 0.6795 .65];
k2_1min = [1e-5, 1.1356e-05, 1.1565e-05, 1e-5];
% k1_doh = .65;
% k2_doh = 1e-5;

% 1 for fine, 1 for k1, 1 for new itr
flagz = boolean([0 1 1]);

fine_wid        = 1.01;
coar_wid        = 1.5;
if flagz(1);k_wid=fine_wid;else k_wid=coar_wid;end
if flagz(2);k_mid=k1_1min(end);else k_mid=k2_1min(end);end
if ~flagz(3);k_mid=ans;end
k_start         = k_mid/k_wid;
k_end           = k_mid*k_wid;
k_itr           = 100;
k_itr           = k_itr+1;
% k_vec           = linspace(k_start,k_end,k_itr);
k_start_log     = log10(k_start);
k_end_log       = log10(k_end);
k_vec           = logspace(k_start_log,k_end_log,k_itr);
time_est        = zeros(k_itr,numXchanges);
rate_est        = zeros(k_itr,numXchanges);
pred_master     = zeros(k_itr,numXchanges);
pred_error      = zeros(k_itr,numXchanges);
rate_pred       = zeros(k_itr,numXchanges);
time_est(:,1)   = meas_master(1);
rate_est(:,1)   = RATE_SEED;
statz           = zeros(k_itr,2);
cnt = 0;

K1 = k1_1min(end);
K2 = k2_1min(end);

wrap = 0;
for k = 1:k_itr
    if flagz(2);K1=k_vec(k);else K2=k_vec(k);end
    cnt = cnt + 1;
    disp(num2str(k));

    for x=2:numXchanges
        %prediction
        pred_master(k,x)=time_est(k,x-1)+rate_est(k,x-1)*SPB*SYNC_PERIOD;
        if pred_master > SPB*SYNC_PERIOD
            while pred_master > SPB*SYNC_PERIOD
                pred_master = pred_master - SPB*(SYNC_PERIOD);
                wrap = wrap + 1;
            end
        end
        rate_pred(k,x) = rate_est(k,x-1);
        %measurement
        pred_error(k,x) = pred_master(k,x) - meas_master(x);
        %adjustment
        time_est(k,x) = pred_master(k,x) - K1*pred_error(k,x);
        rate_est(k,x) = rate_pred(k,x) - K2*pred_error(k,x);
    end
    [mu,sig]=normfit((pred_error(k,1+(end-1000):end)).^2);
    statz(k,:)=[mu,sig];
end


% plot
vu_vec = 1:k_itr;
subplot 211
semilogx(k_vec(vu_vec),statz(vu_vec,1))
title('mean pred_error')
ylabel('samples')
xlabel('K_gain1')
subplot 212
semilogx(k_vec(vu_vec),statz(vu_vec,2))
title('pred_error std')
ylabel('samples')
xlabel('K_gain1')

[pk, pos] = min(abs(statz(:,2)));
k_vec(pos)
statz(pos,2)