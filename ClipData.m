function [test_data] = ClipData(vehicle_type, test_data, reference_motor, start_offset, end_offset, max_freq)
% Clip the data and keep only the part with motor signal higher than
% vehicle type: %0: simquad, 1: solo, 2: simrover, 3: erle rover
% reference_motor. Initial time is the time first reaching reference_motor.
% start_offset: clip offset of starting time. Unit: s
% end_offset: clip offset of ending time. Unit: s

if vehicle_type == 0 || vehicle_type == 1
    refer_idx = find(test_data(:, 14) >= reference_motor, 1);
elseif vehicle_type == 2 || vehicle_type == 3
    refer_idx = find(abs(test_data(:, 16)-0.5) >= reference_motor, 1);
else
    error("unknown vehicle type");
end
reference_time = test_data(refer_idx, 1); % test_data(1, 1)
test_data(:, 1) = test_data(:, 1)-reference_time; % reset start time
%trim data (remove unnecessary parts)
isp = refer_idx + start_offset* max_freq;
iep = find(test_data(:, 14) >= reference_motor,1,'last') - end_offset* max_freq;
test_data = test_data(isp:iep, :);

end

