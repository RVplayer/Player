clear;
close all;
currentFolder = pwd;

model_file = strcat(currentFolder, '/model_sitl_rover.mat');
load(model_file); % read model

%nonlinear greybox model
present(model);
m = model.Parameters(1).Value;
a = model.Parameters(2).Value;
b = model.Parameters(3).Value;
Cx = model.Parameters(4).Value;
Cy = model.Parameters(5).Value;
CA = model.Parameters(6).Value; 
% 
% m = 1.7
% a = 0.5
% b = 0.7
% Cx = 17
% Cy = 17
% CA = 0.1706

%%%%%
% important paraemters for compression
max_freq = 50;
sync_k = 1; %max_freq*10;     % 5 (sec)
is_lowpass = 1;
lowpass_w = 10;
adaptive_order = 1;
if is_lowpass
    load disturb_L_norm_filtered_rover_v2.mat
else
    load disturb_L_norm_rover.mat
end

% L_norm = [4,2]; % for 152 and 153
% L_norm = [0,0];
reference_motor = 0.04; % 1500 +- 40 pwm
%%%%%



%% Read test data
filename = 'Data/Rover/Test6/00000122.csv';
test_data = csvread(filename, 2, 0);
refer_idx = find(abs(test_data(:, 16)-0.5) >= reference_motor, 1);
reference_time = test_data(refer_idx, 1); % test_data(1, 1)
test_data(:, 1) = test_data(:, 1)-reference_time; % reset start time
%trim data (remove unnecessary parts with starting point (s) and end point (s)
    isp = refer_idx + 1 * max_freq;
    iep = find(abs(test_data(:, 16)-0.5) >= reference_motor,1,'last');
    test_data = test_data(isp:iep, :);

NX = 6;    
NY = 6;
NU = 2;
raw_timestamps = test_data(:,1) * 1e-6;    %(s)    time vector
time_us = test_data(:,1);
raw_states = test_data(:,2:13);            % array of state vector: 12 states
raw_motors = test_data(:,14:17);
N = size(test_data,1);                 % size of samples

% extract 6 states  [x y yaw vx vy r]
raw_states(:,1) = raw_states(:,1);
raw_states(:,2) = raw_states(:,2);
raw_states(:,3) = raw_states(:,6);
raw_states(:,4) = raw_states(:,7);
raw_states(:,5) = raw_states(:,8);
raw_states(:,6) = raw_states(:,12);
raw_states(:,7:12) = [];

raw_states(:,3) = wrapTo2Pi(raw_states(:,3));

% convert input signals
% steering (motor1) : 0-0.5 is left turn, 0.5-1 is right turn. => pi
% [-0.5...0.5] *
% throttle (motor3):  
raw_motors(:,1) = (raw_motors(:,1)-0.5)*pi;   
raw_motors(:,2) = raw_motors(:,3)-0.5;
raw_motors(:,3:4) = [];

timestamps = raw_timestamps';
states = raw_states';
motors = raw_motors';

%========== add acceleration AND disturbance data ============
N = N-1;
accel_states = (states(4:6, 2:end)-states(4:6, 1:end-1))./(timestamps(2:end) - timestamps(1:end-1));

% accel_states_bf = [accel_states(1,:) .* cos(states(3,:)) + accel_states(2,:).* sin(states(3,:));...
%     -accel_states(1,:) .* sin(states(3,:)) + accel_states(2,:).* cos(states(3,:));...
%     accel_states(3, :)];
% accel_states = accel_states_bf;
vel_states = states(4:6, :);
vel_states_bf = [vel_states(1,:) .* cos(states(3,:)) + vel_states(2,:).* sin(states(3,:));...
    -vel_states(1,:) .* sin(states(3,:)) + vel_states(2,:).* cos(states(3,:));...
    vel_states(3, :)];

accel_states_bf = (vel_states_bf(:, 2:end) - vel_states_bf(:, 1:end-1)) ./ (timestamps(2:end) - timestamps(1:end-1));
accel_states = accel_states_bf;

vel_states_bf = vel_states_bf(:, 1:end-1);
states = states(:, 1:end-1);
timestamps = timestamps(1:end-1);
time_us = time_us(1:end-1);
motors = motors(:, 1:end-1);


origin_disturb = zeros(4, N);
lowpass_disturb = zeros(4, N);

%========================================

title_name = ["x(north)", "y(east)", "yaw", "vx", "vy", "yaw_rate"]; 
% t, x, y, u
t = timestamps;
x = zeros(NX,N);  %  x(1:12,1) = states(:,1);

%set initial value
x([1:3, 6], 1) = states([1:3, 6], 1);
ef2bf_m = [cos(x(3, 1)), sin(x(3, 1));
        -sin(x(3, 1)), cos(x(3, 1))];
x(4:5, 1) = ef2bf_m * states(4:5, 1);

y = zeros(NY,N);    %model output
y_accel = zeros(3, N);
u = motors;
dx = zeros(NX,N);
accel_log_record = zeros(2, N);
last_log_time = ones(2, 1);
last_log_time(:, 1) = t(1);

for n=1:N-1
    dt = t(n+1) - t(n);
    [dx(:,n),y(:,n)] = rover_m(t(n), x(:,n), u(:,n), m,a,b,Cx,Cy,CA);
    y_accel(:, n) = dx(4:6, n); % record y_accel (body frame).
    disturb_accel = accel_states(:, n) - y_accel(:, n);
    origin_disturb(:, n+1) = [time_us(n+1); disturb_accel];
    %low_pass filtered disturbance
    avg_accel = mean(origin_disturb(2:4, max(1, n+2-lowpass_w):n+1), 2);
    lowpass_disturb(:, n+1) = [time_us(n+1); avg_accel];
    if is_lowpass
       log_reference =  avg_accel;
    else
        log_reference =  disturb_accel;
    end
    lin_acc = norm(log_reference(1:2));
    rot_acc = abs(log_reference(3));
    is_log = islog_disturb(lin_acc, rot_acc ,L_norm, adaptive_order, max_freq, last_log_time, t(n+1));
    accel_log_record(:, n+1) = is_log';
    for i = 1:2
        if is_log(i)
            last_log_time(i) = t(n+1);
        end
    end
    x(:,n+1) = x(:,n) + dx(:,n) * dt; 
    x(3,n+1) = wrapTo2Pi(x(3,n+1)); % wrap yaw to [0,2pi)
    
    if abs(x(4,n+1)) > 30
        x(4,n+1) = sign(x(4,n+1)) * 30;
    end
    
    if abs(x(6,n+1)) > 5
        x(6,n+1) = sign(x(6,n+1)) * 5;
    end
    
%     if n*dt < 2
%           x([1:3, 6], n+1) = states([1:3, 6], n+1);
%           ef2bf_m = [cos(x(3, n+1)), sin(x(3, n+1));
%                     -sin(x(3, n+1)), cos(x(3, n+1))];
%           x(4:5, n+1) = ef2bf_m * states(4:5, n+1);
%     end
    
    
%     k-step ahead estiamtion (sync at every-k loop)
    k = sync_k;
    if mod(n, k) == 0
          x([1:3, 6], n+1) = states([1:3, 6], n+1);
          ef2bf_m = [cos(x(3, n+1)), sin(x(3, n+1));
                    -sin(x(3, n+1)), cos(x(3, n+1))];
          x(4:5, n+1) = ef2bf_m * states(4:5, n+1);
    end

end

%%
% full_disturb = [time_us'; accel_states-y_accel]';
if is_lowpass
    full_disturb = lowpass_disturb';
else
    full_disturb =  origin_disturb';
end
line_acc_norm = vecnorm(full_disturb(: , 2:3)');
rot_acc_norm = abs(full_disturb(:, 4)');
acc_norms = [line_acc_norm; rot_acc_norm]';

actual_log_freqs = zeros(2, N);
w_sz = max_freq;
for i = 1:N
    actual_log_freqs(:, i) = sum(accel_log_record(:, max(1, i-w_sz/2+1):min(i+w_sz/2, N)), 2) / w_sz * max_freq;
end

% L_norm = [max(line_acc_norm), max(rot_acc_norm)]; % save norm
% save('disturb_L_norm.mat', 'L_norm')
% save('disturb_L_norm_filtered_rover_v2.mat', 'L_norm')
%% plot
figure;
for n=1:NY
    if NY > 1
        subplot(NY/3, 3, n);
    end    
    yyaxis left
    plot(timestamps, states(n,:),'k-');     %truth
    hold on;
    plot(t, y(n,:), 'b--');                  %model prediction
    hold on;
    yyaxis right
    area(t, abs(states(n,:)-y(n,:)), 'FaceAlpha', 0.8, 'EdgeColor', 'none');    % deviation
    legend('State', 'Model prediction',  'Error');
end


figure;
for n=1:2
    subplot(1,2,n);
    plot(t, acc_norms(:, n));
    hold on;
    plot (t, ones(size(t)) * L_norm(n));
    yyaxis right
    plot (t, actual_log_freqs(n, :));
end


% return;

%% write data

sync_log_k = max_freq/10;
sync_data = test_data(1:sync_log_k:end, [1:3, 7:9, 13]); % NED frame
T_syn = array2table(sync_data);
T_syn.Properties.VariableNames(1:7) = {'Time_us','x','y', 'yaw', 'V_x', 'V_y', 'Gyro_z'};

writetable(T_syn,[filename(1: end-4) '_syn.csv']);
% The logged data is in NED frame since there is no frame change in rover

T_disturb_lin = array2table(full_disturb(logical(accel_log_record(1, :)'), [1, 2:3]));
T_disturb_rot = array2table(full_disturb(logical(accel_log_record(2, :)'), [1, 4]));

T_disturb_lin.Properties.VariableNames(1:3) = {'Time_us','accel_x','accel_y'};
T_disturb_rot.Properties.VariableNames(1:2) = {'Time_us','angl_accel_z'};

writetable(T_disturb_lin,[filename(1: end-4) '_disturb_lin.csv']);
writetable(T_disturb_rot,[filename(1: end-4) '_disturb_rot.csv']);