%% Tests for which bandwidth parameters give the best precision/accuracy
% Calculates sum squared error and average error among different delay
% values for a variety of bandwidths
%
% Requires Real_SincInit.m and SincGen.m for sinc pulse generation
%
% Joseph Canfield, July 20, 2016
% Last modified : Jan 3, 2017 J.C.
%               : May 20, 2017 T.C.
%                   added axis range to error plot

iterations  = 4999;                         % number of delayed pulses to be generated
max_dly     = 1.5;                          % maximum +/- delay of generated Sinc Pulse
dly_step    = 2*max_dly/(iterations+1);     % Calculate increment of the delay to achieve # of iterations
delay       = -max_dly+dly_step:dly_step:max_dly - dly_step;    % generate delay vector
ampl        = 1;                            % Amplitude of generated pulses
cbw         = 0.5;                          % Normalized Carrier Bandwidth
bwvec       = 0.1:0.01:0.49;                % Vector of Normalized Bandwidth
spb         = 1000;                         % Samples per buffer
ratio       = 10000;                        % Ratio of samples of oversampled template pulse to delayed pulse
fs          = 195.312e3;                    % Sampling frequency
Ts          = 1/fs;                         % Sample Period
fc          = cbw*fs/2;                     % Center frequency of modulated pulse
SNR_dB      = 60;                           % SNR in dB
noDelay     = 0.0;

% Preallocate Vectors
sse         = zeros(1,length(bwvec));
avg         = zeros(1,length(bwvec));
stdev       = zeros(1,length(bwvec));

% time vector of sin/cos modulation
t           = 0:Ts:(spb-1)*Ts;

for b = 1:length(bwvec)
    bw = bwvec(b);
    
    % Initialize oversampled sinc pulse; Real_SincInit.m generates an
    % oversampled pulse that does not include an imaginary component.
    OS_Sinc = Real_SincInit(bw, spb, ratio);
    
    % Generate Undersampled unshifted pulse
    template = SincGen(OS_Sinc,ampl, spb, noDelay);
    
    % Generate Sin and Cos modulated template pulses
    si    = (template.*cos(2*pi*fc*t));
    sq    = (template.*sin(2*pi*fc*t));
    
    % Preallocate vectors
    s  = zeros(iterations,spb);
    recv= zeros(iterations,spb);
    zi  = zeros(iterations,2*spb-1);
    zq  = zeros(iterations,2*spb-1);
    for k = 1:iterations
        % Generate Unmodulated Pulse
        s(k,:)          = SincGen(OS_Sinc,ampl, spb, -delay(k));
        
        % Generate cosine modulation
        recv(k,:)       = (s(k,:).*cos(2*pi*fc*(t-delay(k)*Ts)));
        
        % Add noise
        noise           = randn(1,spb)/sqrt(spb);
        % scale for correct SNR: SNR = S/N0
        recv(k,:)       = recv(k,:)*sqrt(bw*(2/fs)/sum(si.*si));      % scale signal passband to 0dB
        
        % convert SNR to noise variance sig2n
        SNRvar = 10^(SNR_dB/10);
        
        recv(k,:)       = recv(k,:)*sqrt(SNRvar);                        % scale y for correct SNR
        recv(k,:)       = recv(k,:)+noise;
        
        % Perform cross correlation of modulated pulses with template
        zi(k,:) 		= conv(recv(k,:),fliplr(si));
        zq(k,:)         = conv(recv(k,:),fliplr(sq));
    end
    
    % Get maximum of each row of zi
    [pk,pos] = max(abs(zi),[],2);
    
    % Find coarse delay
    int_delay = pos-spb;
    
    % Calculate fractional delay
    frac_delay = zeros(1,iterations);
    for k = 1:iterations
        frac_delay(k) = atan2(zq(k,pos(k)),zi(k,pos(k)))/(pi*cbw);
    end
    
    % Combine integer and fractional components of the delay
    full_delay = int_delay' + frac_delay;
    
    % Calculate statistics
    err = full_delay - delay;
    sse(b) = sum((err).^2);
    avg(b) = mean(abs(err));
    stdev(b)= std(err);
    
    % Plot results for each bandwidth
    figure(54);subplot(211);plot(delay,full_delay);grid on;
    title('Joe''s Code');xlabel('True Delay');
    ylabel('Measured Delay');axis([-1.5 1.5 -1.5 1.5]);
    subplot(212);plot(delay,err,'.');grid on;%axis([-max_dly max_dly -0.05 0.05]);
    title('Error vs. True Delay');xlabel('True Delay');
    ylabel('Error');axis([-1.5 1.5 -0.1 0.1]);
    pause(0.1);
    
    % Print results of each test to terminal
    fprintf('SSE : %d\tAVG : %d with BW : %d\n',sse(b),avg(b),bw);    
end


figure(124)

subplot(211)
plot(bwvec,avg);grid on;
xlabel('Bandwidth');ylabel('Average Error');
title(strcat('Error vs. Bandwidth, CBW = ',num2str(cbw)));

subplot(212)
plot(bwvec,stdev);grid on;
xlabel('Bandwidth');ylabel('Standard Deviation of Error');