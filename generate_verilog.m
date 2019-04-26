%%
clc
clear all

tot_filter = 32;


fprintf( 'assign right_filter_output = \n right_filter_output1 + right_filter_output2 + right_filter_output3 + right_filter_output4' )

for i = 5:4:tot_filter
fprintf('                               \n + right_filter_output%1d + right_filter_output%1d + right_filter_output%1d + right_filter_output%1d'...
        ,i,i+1,i+2,i+3) 
end

fprintf('; \n')

disp(' ')
    
disp(' ')

fprintf( 'assign left_filter_output = \n left_filter_output1 + left_filter_output2 + left_filter_output3 + left_filter_output4' )

for i = 5:4:tot_filter
fprintf('                               \n + left_filter_output%1d + left_filter_output%1d + left_filter_output%1d + left_filter_output%1d'...
        ,i,i+1,i+2,i+3) 
end

fprintf('; \n')