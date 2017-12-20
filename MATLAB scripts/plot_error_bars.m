%%% plot_error_bars.m
% creates error bar plot from .mat file
%be in N210_dev w/ MATLAB_scripts on the path

test = 7

formspec = 'Dat_Dumps/precision/test%i/precision%i.mat';
filename = sprintf(formspec,[test,test]);
load(fullfile(pwd,filename));

error_stats = zeros(del_num,2);
Rldif_stats = zeros(del_num,2);

error_buff = zeros(1,w);
Rldif_buff = zeros(1,w);

for del = 1:del_num
    start_dex=1+w*(del-1); end_dex=w*del;
    error_buff = Rlerror(start_dex:end_dex);
    Rldif_buff = Rldif_clip(start_dex:end_dex);
    [mu sig] = normfit(error_buff);
    error_stats(del,:) = [mu,sig];
    mu = mean(Rldif_buff);
    sig = std(Rldif_buff);

    Rldif_stats(del,:) = [mu sig];
end

figure(3)
clf
subplot 221
plot(del_vec,Rldif_stats(:,1),'.')
ylabel('mean measured offset (Rx samples)')
xlabel('expected offset')
title('mean measured vs expected offset')
subplot 222
errorbar(del_vec/samp_ratio,1e9*error_stats(:,1)/fsRx,1e9*error_stats(:,2)/fsRx,'o')
ylabel('estimation error (ns)')
xlabel('true offset (fractional samples)')
title('mean and std of error')
subplot 212
plot(Rlerror)
a=1;b=length(Rlerror);c=min(Rlerror);d=max(Rlerror);
axis([a,b,c,d])
ylabel('error (Rx samples)')
xlabel('time (Rx samples)')
title('Error over time')

figure(1)
clf
plotspec(Rlerror,1/fsRx)

fprintf('RlMean: %d\nRlSD: %d\n',R1MeanOff,R1SD)

% figure(1)
% title('statistics on measured values and error')
% subplot 221
% plot(del_vec,Rldif_stats(:,1))
% ylabel('mean measured vs expected (Rx samples)')
% subplot 222
% errorbar(del_vec,Rldif_stats(:,1),Rldif_stats(:,2),'o')
% ylabel('mean measured w/ error bars (Rx samples)')
% subplot 223
% plot(del_vec,error_stats(:,1))
% ylabel('error vs expected (Rx samples)')
% subplot 224
% errorbar(del_vec,error_stats(:,1),error_stats(:,2),'o')
% ylabel('error w/ error bars (Rx samples)')
% 
% 
% figure(2)
% plot(Rlerror)
% ylabel('error (Rx samples)')
% xlabel('time (Rx samples)')
% title('Error over time')