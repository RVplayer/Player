% close all;
clear all;
debug = 0;
datahome = 'fusionripper_results/';
trace_name = ["ba-local" "ka-local07" "ka-local31" "ka-highway17" "ka-highway06"];
% attack (crash) threshold (m)   : local 0.895 / 2.405,     high: 1.945 / 2.855
threshold = [2.405, 2.405, 1.115, 2.405, 2.855, 2.855];

% trace_name = ["ba-local"];
% threshold = [2.405];
Ntrace = length(trace_name);
summary = [];

for k = 1:Ntrace
    k=1
    filename = strcat(datahome, 'benign/', trace_name(k), '/ground_truth.csv');
    benign_trace = csvread(filename, 2, 0);  

    benign_trace_time = benign_trace(:,1)-benign_trace(1:1);    % benign_trace(1:1) : init time
    x = (benign_trace(:,4)-benign_trace(1,4))*earthRadius('meters');
    y = (benign_trace(:,5)-benign_trace(1,5))*earthRadius('meters');
    z = (benign_trace(:,6)-benign_trace(1,6))*earthRadius('meters');
    yaw = (benign_trace(:,12)-benign_trace(1,12));
    benign_state = [x,y,z,deg2rad(unwrap(rad2deg(yaw)))];    % (m)
    %figure;plot(benign_trace_time, rad2deg(benign_state(:,4)));  % yaw degree


    %% resample original
    desiredFs = 100; %(default 200Hz, 0.01s)
    Ts = 1/desiredFs;
    [state_resampled, time_resampled] = resample(benign_state,benign_trace_time,desiredFs, 'spline');
    state_resampled = state_resampled(1:end-10,:);
    time_resampled = time_resampled(1:end-10,:);

    figure;
    subplot(1, 2, 1);plot(time_resampled, state_resampled(:,1));  % time x
    subplot(1, 2, 2);plot(time_resampled, state_resampled(:,2));  % time y
    % suptitle("resampled original ground truth (meters)");


    %% attacked
    path = strcat(datahome, 'attack/', trace_name(k), '/');
    files = dir(strcat(path, 'attack*'));
    result_file = strcat(path, 'result');
    total_attack = length(files);
    success = 0;

    total_result_file = strcat(result_file, '.csv');
    fileID = fopen(total_result_file,'w');
    fprintf(fileID,'name, accident, accident_maxdev, max_dev, ');
    fprintf(fileID,"attack_start_from_init, aggr_start_from_init, accident_from_init, ");
    fprintf(fileID,'attack_start_from_init_time, aggr_start_from_init_time, ');
    fprintf(fileID,'accident_time, attack_duration, success_attack_duration, ');
    fprintf(fileID,'crash_x, crash_y, spoofed_gps_x, spoofed_gps_y, pos_diff, start_time_x, start_time_y\n');
    % fclose(fileID);

    Nfile = length(files);
    for i = 1:Nfile
        display([num2str(i), ' / ', num2str(Nfile)])
        close all;
        attack_trace_data = csvread(strcat(path, files(i).name),2,0);
        [fpath, fname, fext] = fileparts(files(i).name);
        remain = fname;
        segments = strings(0);
        while (remain ~= "")
            [token, remain] = strtok(remain, '_-');
            segments = [segments; token];
        end
        direction_str = segments(2);
        ap_start = str2double(segments(3));     % attack profile start in orignal
        aa_start = str2double(segments(4));     % aggressive attack start in orignal
        d = str2double(segments(5));
        f = str2double(segments(6));
%         d = 1.2
%         f = 1.5/
        if strcmp(direction_str, 'rhs')
            direction = 1;
            dir_string = 'rhs';
        else
            direction = -1;
            dir_string = 'lhs';
        end
        result_file_name = strcat('result-', dir_string, '_', string(ap_start), '_', ... 
            string(aa_start), '_', string(d), '_', string(f), '.m');
        result_file_name_csv = strcat('result-', dir_string, '_', string(ap_start), '_', ... 
            string(aa_start), '_', string(d), '_', string(f), '.csv');

        result = strcat(path, result_file_name)


        % time sync to orignal init time
        attack_trace_time = round(attack_trace_data(:,1)-benign_trace(1,1));   %adjusted timestamp
        deviation = attack_trace_data(:,2);                               %deviation
        attack_start_from_init = attack_trace_time(1);            %stage1 start from zero
        aggr_start_from_init = attack_start_from_init + (aa_start - ap_start);         %stage2 start from zero
        at_time_filled = [(0:attack_start_from_init-1), attack_trace_time']';  %filled timestamp
        dev_filled = [zeros(1,attack_start_from_init), deviation']';     %filled devation

        % GPS offset calculation
        GPS_offset = [0,0]; % entire GPS offset (zero to end)
        for j = 1:attack_trace_time(end)    % j is a second
            time_index = find((time_resampled(:,1)) == j);
            current_yaw = state_resampled(time_index, 4);
            if j < attack_start_from_init
                GPS_offset(j+1,:) = [0, 0];
            elseif j < aggr_start_from_init
                GPS_offset(j+1,:) = [d*cos(current_yaw-pi/2)*direction, d*sin(current_yaw-pi/2)*direction];
            else
                ek = j - aggr_start_from_init + 1;
                GPS_offset(j+1,:) = [d*(f^ek)*cos(current_yaw-pi/2)*direction, d*(f^ek)*sin(current_yaw-pi/2)*direction];
            end        
        end   
        
%         [offset_resampled, offset_time_resampled] = resample(GPS_offset, at_time_filled, desiredFs);  
%         [dev_resampled, at_time_resampled] = resample(dev_filled, at_time_filled, desiredFs);
        
        [offset_resampled1, offset_time_resampled1] = resample(GPS_offset(1:attack_start_from_init,:), at_time_filled(1:attack_start_from_init), desiredFs);
        [offset_resampled2, offset_time_resampled2] = resample(GPS_offset(attack_start_from_init+1:end,:), at_time_filled(attack_start_from_init+1:end), desiredFs);
        offset_resampled = [offset_resampled1; offset_resampled2];
        offset_time_resampled = [offset_time_resampled1; offset_time_resampled2];
        
        [dev_resampled1, at_time_resampled1] = resample(dev_filled(1:attack_start_from_init,:), at_time_filled(1:attack_start_from_init), desiredFs);
        [dev_resampled2, at_time_resampled2] = resample(dev_filled(attack_start_from_init+1:end,:), at_time_filled(attack_start_from_init+1:end), desiredFs);
        dev_resampled = [dev_resampled1; dev_resampled2];
        at_time_resampled = [at_time_resampled1; at_time_resampled2];
                
    %     figure;
    %     plot(offset_time_resampled, offset_resampled(:,1), 'r');hold on;
    %     plot(offset_time_resampled, offset_resampled(:,2), 'b');
    %     suptitle("xy (m)");

        if debug == 1
            figure;
            plot(at_time_resampled, dev_resampled, 'r');
            suptitle("deviation (m)");
        end

        N = min(length(time_resampled), length(at_time_resampled));

        attacked_GPS = state_resampled(1:N,1:2) + offset_resampled(1:N,:);
        attacked_MSF = [state_resampled(1:N,1) + dev_resampled(1:N).*cos(state_resampled(1:N,4)-pi/2)*direction ...
           , state_resampled(1:N,2) + dev_resampled(1:N).*sin(state_resampled(1:N,4)-pi/2)*direction];
        true_location = [state_resampled(1:N,1) - dev_resampled(1:N).*cos(state_resampled(1:N,4)-pi/2)*direction ...
           , state_resampled(1:N,2) - dev_resampled(1:N).*sin(state_resampled(1:N,4)-pi/2)*direction];

        lane1 = [state_resampled(:,1) - threshold(k)*1.1.*cos(state_resampled(:,4)-pi/2)*direction ...
           , state_resampled(:,2) - threshold(k)*1.1.*sin(state_resampled(:,4)-pi/2)*direction];
        lane2 = [state_resampled(:,1) + threshold(k)*1.1.*cos(state_resampled(:,4)-pi/2)*direction ...
           , state_resampled(:,2) + threshold(k)*1.1.*sin(state_resampled(:,4)-pi/2)*direction];

        % truncate attacked trace after a crash
        accident = 0;
        attack_duration = 0;
        accident_time_index = 1;1
        if max(deviation) > threshold(k)
            accident = 1;
            success = success + 1;
        end
        for j = 1:N
            attack_duration = time_resampled(j) - time_resampled(aggr_start_from_init);
            if dev_resampled(j) > threshold(k)
                attacked_MSF = attacked_MSF(1:j, :);
                attacked_GPS = attacked_GPS(1:j, :);
                true_location = true_location(1:j, :);
                N = j;
                accident_maxdev = dev_resampled(j);
                accident_time_index = j;
                break;
            end
        end
        
        offset_resampled_noised = [];
        noise_x = smoothdata((rand(N,1)-0.5)*0.01, 'SmoothingFactor', 0.9);
        noise_y = smoothdata((rand(N,1)-0.5)*0.01, 'SmoothingFactor', 0.9);
        offset_resampled_noised(:,1) = offset_resampled(1:N,1) + noise_x;
        offset_resampled_noised(:,2) = offset_resampled(1:N,2) + noise_y;
        dxdt = gradient(offset_resampled_noised(1:N,1)) ./ gradient(offset_time_resampled(1:N));
        dydt = gradient(offset_resampled_noised(1:N,2)) ./ gradient(offset_time_resampled(1:N));
        ddxdt = gradient(dxdt(1:N)) ./ gradient(offset_time_resampled(1:N));
        ddydt = gradient(dydt(1:N)) ./ gradient(offset_time_resampled(1:N));
        
        [max_valuex, attack_startx_idx] = max(ddxdt);
        [max_valuey, attack_starty_idx] = max(ddydt);
        attack_start_time_x = offset_time_resampled(attack_startx_idx);
        attack_start_time_y = offset_time_resampled(attack_starty_idx);
        
        start_time_x = attack_start_time_x;
        start_time_y = attack_start_time_y;
        
        if debug == 1
           figure;
           plot(offset_time_resampled(1:N), offset_resampled_noised(1:N,1)); hold on;
           plot(offset_time_resampled(1:N), offset_resampled_noised(1:N,2)); hold on;
           
           plot(offset_time_resampled(1:N), dxdt);hold on;
           plot(offset_time_resampled(1:N), dydt);hold on;
           
           plot(offset_time_resampled(1:N), ddxdt);hold on;
           plot(offset_time_resampled(1:N), ddydt);hold on;
           figure; plot(offset_time_resampled(1:N), ... 
               gradient(awgn(offset_resampled(1:N,1), 50, 'measured') ./ gradient(offset_time_resampled(1:N))));
           
        end
        

        if debug == 1
            figure;
%             plot(state_resampled(:,1), state_resampled(:,2), '--', 'Color', '#C0C0C0');
            plot(attacked_MSF(1:N,1), attacked_MSF(1:N,2), 'm--');hold on;
            plot(true_location(1:N,1), true_location(1:N,2), 'b-');hold on;
            plot(attacked_GPS(1:N,1), attacked_GPS(1:N,2), 'r--');hold on;
            plot(lane1(:,1), lane1(:,2), 'k-', 'LineWidth', 2);hold on;
            plot(lane2(:,1), lane2(:,2), 'k-', 'LineWidth', 2);hold on;
            legend('MSF', 'Physical location', 'GPS', 'Lanes');
%             suptitle("trajectory (m)");
        end
        

        %% save result
        %========================================================================
        format short;
        fileID = fopen(total_result_file,'a');
        fprintf(fileID,'%s, ',fname);
        fprintf(fileID,'%d, %f, %f, ',accident,accident_maxdev,max(deviation));
        fprintf(fileID,'%d, %d, %d, ',attack_start_from_init,aggr_start_from_init,accident_time_index);
        success_attack_duration = 0;
        if accident == 1
            success_attack_duration = attack_duration;
        end
        fprintf(fileID,'%.4f, %.4f, %.4f, %.4f, %.4f, ', time_resampled(attack_start_from_init), ... 
            time_resampled(aggr_start_from_init), time_resampled(accident_time_index), ...
            attack_duration, success_attack_duration);
        accident_pos = [0,0];
        accident_gps = [0,0];
        if (accident == 1)
            accident_pos = [true_location(accident_time_index,1), true_location(accident_time_index,2)];
            accident_gps = [attacked_GPS(accident_time_index,1), attacked_GPS(accident_time_index,2)];
            accident_msf = [attacked_MSF(accident_time_index,1), attacked_MSF(accident_time_index,2)];
            original_pos = [state_resampled(accident_time_index,1), state_resampled(accident_time_index,2)];
        end
        fprintf(fileID, '%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n', ...
            accident_pos(1), accident_pos(2), accident_gps(1), accident_gps(2), ...
            pdist([accident_pos;accident_gps], 'euclidean'), start_time_x, start_time_y );
        fclose(fileID);
        %=========================================================

        if( debug == 1 )
            figure;plot(accident_pos(1), accident_pos(2), 'ro', ...
                accident_gps(1), accident_gps(2), 'mo', ...
                original_pos(1), original_pos(2), 'ko', ... 
                accident_msf(1), accident_msf(2), 'bo');

            M = 100;
            figure;plot(true_location(accident_time_index-M:accident_time_index,1), true_location(accident_time_index-M:accident_time_index,2), 'b-', ...
                attacked_GPS(accident_time_index-M:accident_time_index,1), attacked_GPS(accident_time_index-M:accident_time_index,2), 'm-', ...
                attacked_MSF(accident_time_index-M:accident_time_index,1), attacked_MSF(accident_time_index-M:accident_time_index,2), 'r-', ... 
                state_resampled(accident_time_index-M:accident_time_index,1), state_resampled(accident_time_index-M:accident_time_index,2), 'k-');
            hold on;
            plot(true_location(accident_time_index,1),true_location(accident_time_index,2), 'b*')
            legend('true', 'gps', 'msf', 'orig', 'accidnt position');

        end
        %% ========================================================================


        if debug == 1
            figure;
            subplot(1, 2, 1);
            plot(time_resampled(1:N), state_resampled(1:N,1), 'k-');hold on;  % time x
            plot(time_resampled(1:N), attacked_MSF(1:N,1), 'r--'); hold on;
            plot(time_resampled(1:N), true_location(1:N,1), 'b--'); hold on;
            plot(time_resampled(1:N), attacked_GPS(1:N,1), 'm--');
            plot(offset_time_resampled(1:N), offset_resampled(1:N,1), 'c--');
            subplot(1, 2, 2);
            plot(time_resampled(1:N), state_resampled(1:N,2), 'k-');hold on;  % time y
            plot(time_resampled(1:N), attacked_MSF(1:N,2), 'r--'); hold on;
            plot(time_resampled(1:N), true_location(1:N,2), 'b--'); hold on;
            plot(time_resampled(1:N), attacked_GPS(1:N,2), 'm--');
            plot(offset_time_resampled(1:N), offset_resampled(1:N,2), 'c--');
            suptitle("positions (original and attacked (MSF output))"); %add ground truth (opposite)
        end

    end
    
    
    summary(k,:) = [success, Nfile];
end




































