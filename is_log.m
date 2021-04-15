function [to_log, desired_freq] = is_log(error,err_th, max_freq, last_log_time, time_now, log_hist)
%is_log: decide whether to log this data point
%   
S = 0.01;                       %TODO: find scale factor "S" 
if last_log_time == time_now
    to_log = 1;
    desired_freq = 0;
    return;
elseif last_log_time < time_now
    desired_freq = error/err_th * max_freq * S; %TODO: scale factor "S"
    log_freq = 1/(time_now-last_log_time);
%     hist_size = size(log_hist,2);
%     log_freq = sum(~isnan(log_hist),2)/hist_size * max_freq;
    log_cond = desired_freq > log_freq;
    
    if log_cond
        to_log = 1;
    else
        to_log = 0;
    end

end

end

