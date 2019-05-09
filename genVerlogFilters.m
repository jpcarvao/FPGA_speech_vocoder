close all 
clear all
clc

shift_freq = 1;

tot_filter = 32;


% fprintf( 'wire [15:0]    right_filter_output , left_filter_output' )
% 
% for i = 1:tot_filter
%     fprintf(', \n               right_filter_output%1d , left_filter_output%1d',i,i) 
% end
% 
% fprintf('; \n')
				

%  -- bandpass
Fs = 8000 ;
T = 1/Fs ;

F_set = [];

% M(f) = 1125*ln(1+f/700)
% F(m) = 700*(exp(m/1125)-1)
% want equispace 300 Hz to 3800 Hz
% M(300) = 401.25 : M(3500)=2016
M = linspace(401.25, 2016, tot_filter);

F_reg = 700*(exp(M/1125)-1);
BW_reg = 0.035*(0.15 ./(F_reg) + 1);

F = 700*(exp(M/1125)-1)/(Fs/2);
%F = logspace(log10(300),log10(3800),32)/(Fs/2) ; % normalized freq
% make the bandpass edges cross at about 50%
BW = 0.035*(0.15 ./(F) + 1); % of freq
% generate the filters
for i=1:length(F)
    %i
    [b{i}, a{i}] = butter(1,[F(i)-F(i)*(BW(i)/2), F(i)+F(i)*(BW(i)/2)] );
    
    genFilter(F_reg(i), BW_reg(i), F_reg(i)*shift_freq ,i)
    
    
%     F_set = [F_set, F(i)-F(i)*(BW(i)/2), F(i)+F(i)*(BW(i)/2)];
    %disp(b{i});
    %disp(a{i});
end
two16 = 2^16;

% plot freq response
figure(1); clf
for i=1:length(F)
    %subplot(2,1,1)
    [h,w] = freqz(b{i}, a{i}, 3000, Fs);
    plot(w,abs(h),'b', 'linewidth',1)
    hold on
    % now the 14-bit truncated filters
    [h,w] = freqz(fix(two16*b{i})/two16, fix(two16*a{i})/two16, 5000, Fs);
    plot(w,abs(h),'r', 'linewidth', 1)
    legend('exact', '2:16', 'Location', 'East')
    %title(['F=', num2str(F), '   BW=', num2str(BW)])
    set(gca, 'xlim', [0 4000])
    set(gca, 'ylim', [0 1])
    xlabel('frequency (Hz)')
    
    
    ylabel('amplitude')
%      subplot(2,1,2)
%      plot(t,out1);
%      xlabel('time (freq of chirp in KHz)')
end
drawnow

figure()
plot(F_set,'*')