clear;
close all;
filename = 'Data/Drone/Test3/00000151.csv';
train_data = csvread(filename, 2, 0);  
% states [x y z roll pitch yaw vx vy vz p q r]
% output = states
% input [m1 m2 m3 m4]
NX = 12;    
NY = 12;
NU = 4;

%reset start time
train_data(:,1) = train_data(:,1) - train_data(1,1);
%trim data (remove unnecessary parts with starting point (s) and end point (s)
    sp = 10; % starting skip (s)
    ep = 20; % end skip (s)
    isp = find(train_data(:,1) * 1e-6 > sp, 1);
    iep = find(train_data(:,1) > train_data(end,1) - ep * 1e6, 1);
    train_data = train_data(isp:iep, :);
    
raw_timestamps = train_data(:,1) * 1e-6;    %(s)    time vector
raw_states = train_data(:,2:13);            % array of state vector: 12 states
% converting: motor_input = (pwm - 1100)/900 
% motor signal [0..1] --> motor speed (PWM servo) [0...2000]
raw_motors = (((train_data(:,14:17)*1000)+1000)-1100)/900;   %(((0.5595*1000)+1000)-1100)/900=0.51
raw_motors(raw_motors<0) = 0;
raw_motors(raw_motors>1) = 1;
raw_N = size(train_data,1);                 % size of samples

%%%Preprocessing ================================

%========== to ENU ==============
% x <-> y, vx <-> vy
temp = raw_states(:,1); 
raw_states(:,1) = raw_states(:,2);
raw_states(:,2) = temp;
temp = raw_states(:,7); 
raw_states(:,7) = raw_states(:,8);
raw_states(:,8) = temp;

% z <-> -z, vz <-> -vz
raw_states(:,3) = -raw_states(:,3);
raw_states(:,9) = -raw_states(:,9);

% pitch <-> -pitch yaw = (-yaw + pi/2)
raw_states(:,5) = -raw_states(:,5);
raw_states(:,6) = mod(-raw_states(:,6)+pi/2, 2*pi);

% q <-> -q r <-> -r
raw_states(:,11) = -raw_states(:,11);
raw_states(:,12) = -raw_states(:,12);



%%resample (for uniform sampling time)
desiredFs = 400; %(default 400Hz)
Ts = 1/desiredFs;
[res_states, res_timestamps] = resample(raw_states,raw_timestamps,desiredFs);
[res_motors, res_timestamps] = resample(raw_motors,raw_timestamps,desiredFs);
N = size(res_timestamps,1);

%plot (original and sampeled)
title_name = ["x(east)", "y(north)", "z(up)", "roll", "pitch", "yaw", "vx", "vy", "vz", "p", "q", "r"]; 

figure;
for n=1:NY
    subplot(NY/3, 3, n);
    plot(raw_timestamps,raw_states(:,n),'b.-');
    hold on;
    plot(res_timestamps, res_states(:,n),'r-');
    legend('Original','Resampled');
    title(title_name(n));
end

%%convert to column vectors
timestamps = res_timestamps';
states = res_states';
motors = res_motors';
%%%============================================

%% ==============================================================
%% parameters (default)
g = 9.80665;   % gravity acceleration constant (m/s^2)

arm_scale = deg2rad(5000);
yaw_scale = deg2rad(400);

thetas = [deg2rad(45), deg2rad(-135), deg2rad(-45), deg2rad(135)];

m = 1.5;
I_x = 0.015; 
I_y = 0.015; 
I_z = 0.015;

throttle_hover = 0.51;

K_T = m*g / (4*throttle_hover);

K_Q = yaw_scale * I_z;

a =  sin(thetas(1)) * arm_scale * I_x / K_T;  
b =  cos(thetas(1)) * arm_scale * I_y / K_T;    
c = -cos(thetas(4)) * arm_scale * I_y / K_T;    
d =  sin(thetas(4)) * arm_scale * I_x / K_T;    

%====================================
% test the model implementation
t = timestamps;
x = zeros(NX,N);    x(1:12,1) = states(:,1);
dx = zeros(NX,N);
y = zeros(NY,N);
u = motors;
err = zeros(NX, N);
global frame_height;
frame_height = 0.1;
for n=1:N-1
    dt = t(n+1) - t(n);
    [dx(:,n),y(:,n)] = quadrotor_m(t(n), x(:,n), u(:,n), a,b,c,d, m, I_x, I_y, I_z, K_T, K_Q);
    x(:,n+1) = x(:,n) + dx(:,n) * dt; 
    x(6,n+1) = mod(x(6,n+1), 2*pi);
    %========= on ground check ==========
    if on_ground(x(3, n+1), frame_height)
        x(3, n+1) = frame_height; % z ;
        x(4:5,n+1) = 0; % roll = pitch = 0;
        x(7:8, n+1) = 0; % vx = vy = 0;
        x(10:12, n+1) = 0; %pqr = 0;
        if x(9, n+1) < 0
            x(9, n+1) = 0; %vz = 0;
        end
    end

%     sychronize for the first 10 seconds
    if n * dt < 10
        x(10:12,n+1) = states(10:12,n+1);
        x(4:6,n+1) = states(4:6,n+1);
    end
    
    %k-step ahead estiamtion (sync at every-k loop)
%     k = 3 * desiredFs;
%     if mod(n, k) == 0
%         x(:,n+1) = states(:,n+1);
%     end
    
    err(:, n) = abs(y(:, n) - states(:, n));
end

% err_mean = mean(err, 2);
% err_max = max(err, [], 2);
% err_quants = quantile(err', [0.25, 0.75])';
% err_thresh = err_quants(:, 2) + 1.5 * (err_quants(:, 2)-err_quants(:, 1));
figure;
for n=1:NY
    if NY > 1
        subplot(NY/3, 3, n);
    end
    yyaxis left
    plot(timestamps, states(n,:),'r.-');
    hold on;
    plot(t, y(n,:), 'b-');  
    yyaxis right
    area(t, err(n, :), 'FaceAlpha', 0.8, 'EdgeColor', 'none');
    legend('Resampled', 'Model', 'Error');
    title(title_name(n));
end

% return

%====================================
%% Nonlinear grey-box model - idnlgrey
%
% data = iddata(states, motors, 'SamplingInstants', timestamps); %non-uniform sampling
data = iddata(states', motors', Ts);    % uniform sampling (Ts)
Filename       = 'quadrotor_m';                % File describing the model structure.
Order          = [NY NU NX];               % Model orders [ny nu nx].
Parameters    = [a; b; c; d; m; I_x; I_y; I_z; K_T; K_Q];   % Initial parameter vector.
InitialStates = x(:,1);
nlgr_m    = idnlgrey(Filename, Order, Parameters, InitialStates);   %Nonlinear grey-box model

%% Parameter setting
% 
nlgr_m.Parameters(1).Fixed = true;      %abcd
nlgr_m.Parameters(2).Fixed = true;      
nlgr_m.Parameters(3).Fixed = true;
nlgr_m.Parameters(4).Fixed = true;
nlgr_m.Parameters(5).Fixed = true;      %weight
% nlgr_m.Parameters(6).Fixed = true;      %Ix
% nlgr_m.Parameters(7).Fixed = true;      %Iy
% nlgr_m.Parameters(8).Fixed = true;      %Iz
% nlgr_m.Parameters(9).Fixed = true;     %thrust const  **
% nlgr_m.Parameters(10).Fixed = true;    %torque_const  **
%nlgr_m.Parameters(11).Fixed = true;    %K_m  **
%nlgr_m.Parameters(12).Fixed = true;    %alpha  **
% %-----------------------------------------------------------
% nlgr_m.Parameters(1).Minimum = 0.10;      %abcd [0.1..0.26]
% nlgr_m.Parameters(2).Minimum = 0.10;      
% nlgr_m.Parameters(3).Minimum = 0.10;      
% nlgr_m.Parameters(4).Minimum = 0.10; %0.18;      
% nlgr_m.Parameters(5).Minimum = 0;       %weight     
% nlgr_m.Parameters(6).Minimum = 0;         %I_x
% nlgr_m.Parameters(7).Minimum = 0;         %I_y      
% nlgr_m.Parameters(8).Minimum = 0;         %I_z   
nlgr_m.Parameters(9).Minimum = 0;       %th const
nlgr_m.Parameters(10).Minimum = 0;      %torque_const
% nlgr_m.Parameters(11).Minimum = 0;      %K_m
% nlgr_m.Parameters(12).Minimum = 0;      %alpha

% nlgr_m.Parameters(1).Maximum = 0.30; %0.22;      %abcd [0.1..0.26]
% nlgr_m.Parameters(2).Maximum = 0.30; %0.16;
% nlgr_m.Parameters(3).Maximum = 0.30; %0.16;
% nlgr_m.Parameters(4).Maximum = 0.30; %0.20;
% nlgr_m.Parameters(5).Maximum = 1.6;%1.4;         %weight     
% nlgr_m.Parameters(6).Maximum = 0.1;        %I_x
% nlgr_m.Parameters(7).Maximum = 0.1;  
% nlgr_m.Parameters(8).Maximum = 0.2;  
% nlgr_m.Parameters(9).Maximum = 0.6;         %th_const
% nlgr_m.Parameters(10).Maximum = 0.001;      %torque_const
% nlgr_m.Parameters(11).Maximum = 0.001;      %K_m
% nlgr_m.Parameters(12).Maximum = 0.001;      %alpha

nlgr_m = setpar(nlgr_m, 'Name', {'a' 'b' 'c' 'd' 'weight' 'Ix' 'Iy' 'Iz' 'thrust_const' 'torque_const'});
nlgr_m = setpar(nlgr_m, 'Unit', {'m' 'm' 'm' 'm' 'kg' 'kg*m^2' 'kg*m^2' 'kg*m^2' '%', 'N*(rad/s)^2'});
% nlgr_m.SimulationOptions.AbsTol = 1e-10;
% nlgr_m.SimulationOptions.RelTol = 1e-10;

%% System Identification
%%SI options ===================================================
%
opt = nlgreyestOptions;
opt.Display = 'on';
%opt.GradientOptions.DiffMaxChange              % default: Inf
% opt.GradientOptions.DiffMinChange = 1e-3;    % defailt: 0.01*sqrt(eps) = 1.4901e-10
%opt.GradientOptions.DiffScheme = 'auto';       % default: auto, 'Central approximation', 'Forward approximation', 'Backward approximation'
% opt.GradientOptions.GradientType = 'auto';      % default: auto, 'Basic', 'Refined'
opt.SearchMethod = 'gn';                        % default: auto(lsqnonlin), gn, gna, lm, grad, fmincon
% opt.SearchOption.Tolerance = 1e-10;     %FunctionTolerance (1e-5) - gn
% opt.SearchOption.Algorithm = 'interior-point';     % interior-point, sqp, active-set, trust-region-reflective, 
% opt.SearchOption.TolFun = 1e-10;      %FunctionTolerance (1e-5)
% opt.SearchOption.TolX = 1e-10;        %StepTolerance (1e-6)
opt.SearchOption.MaxIter = 100;
% opt.SearchOption.Advanced.GnPinvConst = 10000000;
%opt.Regularization.Nominal = 'model';          %default: 'zero'

%--------------------------------------------
% outputweight = eye(NX);
% outputweight(1,1) = 1;
% outputweight(2,2) = 1;
% outputweight(3,3) = 1;
% outputweight(4,4) = 1000;
% outputweight(5,5) = 1;
% outputweight(6,6) = 1;
% opt.OutputWeight = outputweight;

model = nlgreyest(data, nlgr_m, opt);    %Estimate nonlinear grey-box model parameters
% model = nlgr_m;
% present(model);
% return

%%===================================================
%% plot results
%%

% sim vs prediction
input_data = iddata([], motors', Ts);
ys = sim(model,input_data);
K = 10; % K-step-ahead prediction
yp = predict(nlgr_m, data, K);

figure;
for n=1:NY
    if NY > 1
        subplot(NY/3, 3, n);
    end
    plot(timestamps, data.y(:,n), 'k-');
    hold on; 
    plot(timestamps, ys.y(:,n),'r.-');
    hold on;
    plot(timestamps, yp.y(:,n), 'b.');  
    legend('truth', 'simulated', 'predicted');
    title(title_name(n));
end

figure; 
compare(model, data);