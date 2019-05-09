function genFilter(freq, BW, new_freq , count)

    % IIR header test pgm

    % at Fs=48kHz normalized F=1 at 24kHz
    shifted = round((new_freq*(2^32))/(48000));
%     freq = 4800;
    F = freq/24000;
%     BW = 0.15;
    [b,a] = butter(1,[F-F*(BW/2), F+F*(BW/2)] );

%     disp('low: ')
%     disp((F-F*(BW/2))*24000)
% 
%     disp('high: ')
%     disp((F+F*(BW/2))*24000)

    freqz(b,a)
%%
    a = -a ; % makes it easier to use an MAC
    disp(' ')
    fprintf('//Filter%2d Right \n',count)
    fprintf('//Filter%2d: frequency=%f \n',count,F(1)*24000)
    fprintf('//Filter%2d: BW=%f \n',count,BW(1))
    fprintf('//Filter%2d: shifted freq=%f \n',count,new_freq)
    sorder = 2;
    scstr = ['IIR2_18bit_fixed filter',  num2str(count),  '_RIGHT('];

    fprintf('%s \n',scstr);
    fprintf('     .audio_out (right_filter_output%1d), \n',count)
    fprintf('     .audio_in (right_audio_input), \n')
    fprintf('     .freq_sw (32''d%d <<< pitch_shift), \n',shifted)
    for i=1:length(b)
        if b(i)>=0
            fprintf('     .b%1d (18''sd%d), \n', i,fix(2^16*b(i)) ) ;
        else
            fprintf('     .b%1d (-18''sd%d), \n', i, fix(-2^16*b(i)) );
        end
    end

    for i=2:length(a)
        if a(i)>=0
            fprintf('     .a%1d (18''sd%d), \n', i, fix(2^16*a(i)) )
        else
            fprintf('     .a%1d (-18''sd%d), \n', i, fix(-2^16*a(i)) )
        end
    end
    fprintf('     .state_clk(CLOCK_50), \n');
    fprintf('     .audio_input_ready(audio_input_ready), \n');
    fprintf('     .reset(reset) \n');
    fprintf(') ; //end filter \n');

    disp(' ')
%%    
    disp(' ')
    fprintf('//Filter%2d Left \n',count)
    fprintf('//Filter%2d: frequency=%f \n',count,F(1)*24000)
    fprintf('//Filter%2d: BW=%f \n',count,BW(1))
    fprintf('//Filter%2d: shifted freq=%f \n',count,new_freq)
    sorder = 2;
    scstr = ['IIR2_18bit_fixed filter',  num2str(count),  '_LEFT('];

    fprintf('%s \n',scstr);
    fprintf('     .audio_out (left_filter_output%1d), \n',count)
    fprintf('     .audio_in (left_audio_input), \n')
    fprintf('     .freq_sw (32''d%d <<< pitch_shift), \n',shifted)
    for i=1:length(b)
        if b(i)>=0
            fprintf('     .b%1d (18''sd%d), \n', i,fix(2^16*b(i)) ) ;
        else
            fprintf('     .b%1d (-18''sd%d), \n', i, fix(-2^16*b(i)) );
        end
    end

    for i=2:length(a)
        if a(i)>=0
            fprintf('     .a%1d (18''sd%d), \n', i, fix(2^16*a(i)) )
        else
            fprintf('     .a%1d (-18''sd%d), \n', i, fix(-2^16*a(i)) )
        end
    end
    fprintf('     .state_clk(CLOCK_50), \n');
    fprintf('     .audio_input_ready(audio_input_ready), \n');
    fprintf('     .reset(reset) \n');
    
    fprintf(') ; //end filter \n');

    disp(' ')

end







% 
%  wire signed[15:0] debug_right9, debug_left9;
%  wire signed[15:0] debug_right8, debug_left8;
%  wire 		[15:0] debug_right7, debug_left7;
%  wire signed[15:0] debug_right6, debug_left6;
%  wire signed[15:0] debug_right5, debug_left5;
%  
% //Filter 3 Right 
% //Filter 3: frequency=0.016542 
% //Filter 3: BW=0.035013 
% IIR2_18bit_fixed filter3_RIGHT( 
%      .audio_out (right_filter_output3), 
%      .audio_in (right_audio_input), 
% 	  .freq_sw(SW[9:0]),
%      .b1 (18'sd59), 
%      .b2 (18'sd0), 
%      .b3 (-18'sd59), 
%      .a2 (18'sd130776), 
%      .a3 (-18'sd65416), 
%      .state_clk(CLOCK_50), 
%      .audio_input_ready(audio_input_ready), 
%      .reset(reset),
% 	  .voice_in_abs( debug_left9),
% 	  .voice_low_new(debug_left8),
% 	  .wave(debug_left7),
% 	  .modulated_voice(debug_left6),
% 	  .f1_mac_new(debug_left5)
% ) ; //end filter 
%  
% 
%  
% //Filter 3 Left 
% //Filter 3: frequency=0.016542 
% //Filter 3: BW=0.035013 
% IIR2_18bit_fixed filter3_LEFT( 
%      .audio_out (left_filter_output3), 
%      .audio_in (left_audio_input), 
% 	  .freq_sw(SW[9:0]),
%      .b1 (18'sd59), 
%      .b2 (18'sd0), 
%      .b3 (-18'sd59), 
%      .a2 (18'sd130776), 
%      .a3 (-18'sd65416), 
%      .state_clk(CLOCK_50), 
%      .audio_input_ready(audio_input_ready), 
%      .reset(reset),
% 	  .voice_in_abs( debug_right9),
% 	  .voice_low_new(debug_right8),
% 	  .wave(debug_right7),
% 	  .modulated_voice(debug_right6),
% 	  .f1_mac_new(debug_right5) 
% ) ; //end filter 
%  