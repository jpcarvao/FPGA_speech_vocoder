% filters used for speech synthesis
% http://uvafon.hum.uva.nl/david/ma_ssp/2010/Klatt-1980-JAS000971.pdf
% impulse invariant design

clear all

Fs = 8000 ;
T = 1/Fs ;

%
%  voice_input = wavread('Voice_001.wav');
%  voice_input = resample(voice_input,1,2);

voice_input = audioread('test1.wav');

voice_input = voice_input/max(voice_input);
%sound(voice_input);
figure(2);clf
subplot(2,1,1)
pwelch(voice_input)
title('Original')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  -- bandpass
% M(f) = 1125*ln(1+f/700)
% F(m) = 700*(exp(m/1125)-1)
% want equispace 300 Hz to 3800 Hz
% M(300) = 401.25 : M(3500)=2016
M = linspace(401.25, 2016, 32);
F = 700*(exp(M/1125)-1)/(Fs/2);
%F = logspace(log10(300),log10(3800),32)/(Fs/2) ; % normalized freq
% make the bandpass edges cross at about 50%
BW = 0.035*(0.15 ./(F) + 1); % of freq
% generate the filters
for i=1:length(F)
    %i
    [b{i}, a{i}] = butter(1,[F(i)-F(i)*(BW(i)/2), F(i)+F(i)*(BW(i)/2)] );
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

disp(' ')
fprintf('//Filter: frequency=%f \n',F(1))
fprintf('//Filter: BW=%f \n',BW(1))
sorder = 2;
scstr = 'IIR2 filter(';

fprintf('%s \n',scstr); 
fprintf('     .audio_out (your_out), \n')
fprintf('     .audio_in (your_in), \n')
for i=1:length(b{1})
    if b{1}(i)>=0
        fprintf('     .b%1d (18''h%s), \n', i, dec2hex(fix(2^16*b{1}(i)))) ;
    else
        fprintf('     .b%1d (-18''h%s), \n', i, dec2hex(fix(2^16*-b{1}(i))));
    end
end

for i=2:length(a{1})
    if a{1}(i)>=0
        fprintf('     .a%1d (18''h%s), \n', i, dec2hex(fix(2^16*a{1}(i))))
    else
        fprintf('     .a%1d (-18''h%s), \n', i,dec2hex(fix(2^16*-a{1}(i))))
    end
end
fprintf('     .state_clk(CLOCK_50), \n');
fprintf('     .audio_input_ready(audio_input_ready), \n');
fprintf('     .reset(reset) \n');
fprintf(') ; //end filter \n');

disp(' ')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% now use these to filter/synth voice
% use 1-pole lowpass with abs value input to get average 
% in each filter band. Time const ~ 64 samples
figure(2)
band = 10;
 for i=1:length(F)
     voice_filter(:,i) = filter(b{i}, a{i}, voice_input);
 end
 hold on
 %plot(voice_filter(:,band));
 %energy -- filter_out[i] = filter_out[i] + ((absfix14(output) - filter_out[i])>>6); 
 for i=1:length(F)
     voice_filter(:,i) = abs(voice_filter(:,i));
     % voice_filter(:,i) = filter(1/64, [1 -1+1/64], voice_filter(:,i)); %
     % 8 mSec
     voice_filter(:,i) = filter(1/128, [1 -1+1/128], voice_filter(:,i)); 
 end
hold on
 %plot(voice_filter(:,band), 'r');

voice_out = zeros(size(voice_input));
t = (1/Fs:1/Fs:length(voice_input)/Fs)';

%get N loudest bands
% N = 25 ;
% for j=1:length(voice_out)
%     for i=1:N
%         [maxF,posF] = max(voice_filter(j,:),[],2);
%         voice_filter(j,posF) = 0 ; % eliminate the biggest entry for the
%         next loop
%         voice_out(j) = voice_out(j) + maxF * sin(2*pi*F(posF)*4000*t(j));
%     end
% end

% reduce data rate for each filter output
% must be 64, 128 or 256 and must match table_interval below
filter_sample_interval = 0.016*Fs; % seconds 16 mS is 128 sample times
filter_sample_quantization = 255 ; % levels
for i = 1:length(F)
    vf = voice_filter(1,i);
    for j = 1:length(voice_filter(:,i))
        if mod(j,filter_sample_interval)>0
            voice_filter(j,i) = vf;
        else
            vf = voice_filter(j,i);
        end
    end   
end

%normalize and truncate
mm = max(max(voice_filter));
for i = 1:length(F)
    voice_filter(:,i)=fix(filter_sample_quantization*voice_filter(:,i)/mm);
end

%make a textfile with PIC XC32 source code in it.
fname='Bruce_digits_spectral_15_340_3400.h';
fid = fopen(fname,'w');
%static constant means to store it in flash
% store 25 filter amps / time step
count = 0;
fprintf(fid,'static const unsigned char AllDigits[]={\r');
for i=1:128:length(voice_filter(:,1))
    fprintf(fid,' %5d,\r',voice_filter(i,:));
    count = count + length(F);
end
fprintf(fid,' -1\r');
fprintf(fid,'};\r');
fprintf(fid,'#define table_count %d\r', count);
% allowed values 63, 127, 255 MUST match filter_sample_interval-1
fprintf(fid,'#define table_interval %d\r', 127);
% frequency table
fprintf(fid,'#define NUMSINE %d\r', length(F));
fprintf(fid,'int F[]={ \r');
fprintf(fid,' %5i,\r',fix(F*Fs/2));
fprintf(fid,'};\r');
fclose(fid);

length(voice_input)/count

% sum sine waves
for i = 1:length(F)
      voice_out = voice_out + voice_filter(:,i) .* sin(2*pi*F(i)*4000*t);
%           fix(filter_sample_quantization*voice_filter(:,i)/max(voice_filter(:,i))) .* ...
%           sin(2*pi*F(i)*4000*t);
            
end

voice_out = voice_out/max(voice_out);
subplot(2,1,2)
pwelch(voice_out)
title('Reconstructed')
sound(voice_out, Fs);
%wavwrite(voice_out, 'Fund_the_arts_30_filters.wav')
%%%%%%% end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%