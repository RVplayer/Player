% close all;
clear all;
debug = 0;
datahome = 'fusionripper_results/';
trace_name = ["ba-local" "ka-local07" "ka-local31" "ka-highway17" "ka-highway06"];

Ntrace = length(trace_name);
result = table;
means = [];
for i = 1:Ntrace
    filename = strcat(datahome, 'attack/', trace_name(i), '/result.csv');
    data = readtable(filename);
    N = height(data);
    
    idx = (data.accident == 1);
    accident_data = data(idx,:); % accident case among the entire
    
    success = table2array(data(:,2));
    accident_maxdev = table2array(accident_data(:,3));
    max_dev = table2array(data(:,4));
    success_attack_duration = table2array(accident_data(:,12));
    pos_diff = table2array(accident_data(:,17));
    
    truth_start = table2array(accident_data(:,5));
    start_x = table2array(accident_data(:,18));
    start_y = table2array(accident_data(:,19));
    start_xy = (start_x + start_y)/2;
    start_time_error = abs(truth_start-start_xy);
    
%     S = length(find (success == 1));
    S = height(accident_data);

    tname_index = repelem(i, S)';
%     pos_diff = pos_diff(pos_diff~=0);
    
    means_pos_diff(i) = mean(pos_diff);
    means_start_time_error(i) = mean(start_time_error);
    
    T = table(tname_index, pos_diff, success_attack_duration, start_time_error);
    result = [result; T];
end

label = categorical(result.tname_index, 1:Ntrace, trace_name);
figure;
boxchart(label, result.pos_diff);%, 'Notch', 'on');
hold on;
plot(means_pos_diff, '-o');
plot(rand(1,6)*0.2, '-x'); %liadr (error <20cm)
plot(zeros(6,1), '-s'); %truth
xlabel("Attack Traces");
ylabel("Position Difference (m)")
legend("Diff", "Mean GPS (Spoofed)", "Mean Lidar", "Ground Truth")
hold off;


mean_pos_diff = mean(result(result.tname_index == 1,:).pos_diff);
mean_attack_duration = mean(result(result.tname_index == 1,:).success_attack_duration);
mean_start_time_error_total = mean(result(result.tname_index == 1,:).start_time_error);
