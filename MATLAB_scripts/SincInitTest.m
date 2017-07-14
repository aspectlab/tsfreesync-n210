%% SincInitTest.m
% script tests SincInit.m under various delay conditions

spbTx       = 1000;
fsRx        = 5e6;
fsTx        = 250e3;
ratio       = fsRx/fsTx;
OSRatio     = 10e3;
spbRx       = spbTx*ratio;
numPulses   = 1000;
dataLen     = numPulses*spbRx;

fNyqTx      = fsTx/2;
TsRx        = 1/fsRx;

ampl        = 1.0;
cbw         = 1/2;
sbw         = (15/16)*cbw;              % Sinc use (9/10) of channel width
W0          = 2*pi*fNyqTx*cbw;
w0          = W0*TsRx;

meanDelay   = 0;
delayRng    = 40;
% actDel      = linspace(meanDelay-delayRng, meanDelay+delayRng,numPulses);

actDel      = randn(1,numPulses) * delayRng + meanDelay;

% actDel      = 1./logspace(0,2.999999999,numPulses/2);
% actDel      = [-actDel(2:end),0,fliplr(actDel)];

% actDel      = zeros(1,numPulses);

SNdB        = inf;
rand_tog    = 1;
randDel     = rand_tog * randn(1,numPulses);
% randDel     = 0*ones(1,numPulses);
delVec      = actDel + randDel;
noDelay     = 0; 

%% init & gen sinc template
COS_Sinc    = SincInit(sbw,cbw,spbTx,OSRatio);
ROS_Sinc    = real(COS_Sinc);
temp        = SincGen(COS_Sinc,ampl,spbRx,noDelay);
si          = real(temp);
sq          = imag(temp);
%% generate sample data

Apulse      = zeros(numPulses,spbRx);
Bpulse      = zeros(numPulses,spbRx);
SNR         = 10^(SNdB/10);

for n = 1:numPulses
    Apulse(n,:)     = SincGen(ROS_Sinc,ampl,spbRx,randDel(n))...
                        + randn(1,spbRx)/sqrt(spbRx)/sqrt(SNR);

                    
    Bpulse(n,:)     = SincGen(ROS_Sinc,ampl,spbRx,delVec(n))...
                        + randn(1,spbRx)/sqrt(spbRx)/sqrt(SNR);

end

Rldif0      = zeros(1,numPulses);
Rldif1      = zeros(1,numPulses);
Csdif0      = zeros(1,numPulses);
Csdif1      = zeros(1,numPulses);



for n = 1:numPulses
    
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
%     plot(max(max(Apulse(n,:)),max(Bpulse(n,:)))*si)
%     hold off
%     [n,posA,posB]
    
    coarse_delayA = posA-spbRx;
    coarse_delayB = posB-spbRx;
    coarse_delay  = posA - posB;
    
%     [n,actDel(n)]
%     figure(99)
%     plot(zqA)
%     figure(100)
%     plot(zqB)
    
    thetaA = atan2(zqA(posA),ziA(posA));
    thetaB = atan2(zqB(posB),ziB(posB));
    fine_delay = (thetaA - thetaB)/(w0);
    
    corr1 = abs(xcorr(si,Bpulse(n,:)));
    [~,coarse_offset1]=max(corr1);
    coarse_offset1=coarse_offset1-spbRx;
    
    y=corr1(coarse_offset1+spbRx-1:coarse_offset1+spbRx+1);
    fine_offset1=(y(1)-y(3))/2/(y(1)-2*y(2)+y(3));
    
    % store offsets

    Rldif0(n) = coarse_offset1 + fine_offset1;
    Rldif1(n) = coarse_delay + fine_delay;
    Csdif1(n) = coarse_delay;
    Csdif0(n) = coarse_offset1;
end


%% Calculate Statistics (in units of actual time, not in fractional samples)
R1MeanOff1 = mean(Rldif1)*(1/fsRx);
R1SD1 =      std(Rldif1)*(1/fsRx);
R1MeanOff0 = mean(Rldif0)*(1/fsRx);
R1SD0 =      std(Rldif0)*(1/fsRx);
%% Print Stats
fprintf('St. Dev:\t%1.3d ns\nMean:\t%1.3d ns\n\n',R1SD1*1e9,R1MeanOff1*1e9);
fprintf('St. Dev:\t%1.3d ns\nMean:\t%1.3d ns\n\n',R1SD0*1e9,R1MeanOff0*1e9);
%% Plot Stats
figure(333)
hold off
plot(actDel,actDel,'c')
hold on
plot(actDel,Rldif1,'b.')
% plot(actDel,Rldif0,'r.')
% plot(actDel,Csdif1,'b-')
% plot(actDel,Csdif0,'r-')
title('arctan estimator')
xlabel('actual delay')
ylabel('measured offset (in fractional Rx samples)')
grid on
hold off

figure(334)
hold off
plot(actDel,'c')
hold on
plot(Rldif1,'b.')
% plot(Rldif0,'r.')
plot(Csdif1,'b-')
% plot(Csdif0,'r-')
title('arctan estimator')
xlabel('actual delay')
ylabel('measured offset (in fractional Rx samples)')
grid on
hold off

error0 = mean(abs(Rldif0 - actDel));
error1 = mean(abs(Rldif1 - actDel));
[error0,error1]


% figure(3)  % check to make sure we're not half-way out of phase
% plot(abs(ChanA(1:round(length(ChanA)/plot_scale))))
% hold on
% plot(abs(ChanB(1:round(length(ChanA)/plot_scale))),'r')
% hold off

% figure(501)
% plot(Apulse(501,:))
% hold on
% plot(Bpulse(501,:))
% plot(si)
% plot(sq)
% hold off