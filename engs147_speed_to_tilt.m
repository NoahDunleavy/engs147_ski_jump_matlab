%Noah Dunleavy
%ENGS147 - Change in Speed to Change in Angle
clc; clear; close all;
%% Read in Data
filepath = "C:\Users\noahd\OneDrive\Desktop\ENGS 147\Final Project\hanging_test_v1.txt";
arduino_parse = parse_arduino_log(filepath);
num_tests = length(arduino_parse);



for test_num = 1:num_tests
    parse = arduino_parse(test_num);

    step_pwm = parse.test_info(1).value;
    dt_us = parse.test_info(2).value;
    time_to_step_ms = parse.test_info(3).value;
    step_duration_ms = parse.test_info(4).value;

    time_stamp_sec = (parse.data(:, 1) - parse.data(1, 1)) / 1000000;%normalize to 0, scale to seconds
    wheel_speed_rad_sec = parse.data(:, 2);
    command_pwm = parse.data(:, 3);
    pitch_rad = parse.data(:, 4);
    %Do a fix for the fact that car is upside down
    pitch_rad = pitch_rad + pi;
    flipped_ndx = pitch_rad > pi;
    pitch_rad(flipped_ndx) = pitch_rad(flipped_ndx) - 2*pi;

    ang_vel = parse.data(:, 5);

    figure('Name', sprintf('PWM: %g Test', step_pwm));

    %Wheel Speed
    subplot(2, 1, 1);
    xlabel('Time [sec]')
    yyaxis left
    plot(time_stamp_sec, wheel_speed_rad_sec, 'LineWidth', 1.3);
    ylabel('Wheel Speed [rad/sec]')
    yyaxis right
    plot(time_stamp_sec, command_pwm, 'LineWidth', 1.5);
    ylabel('Command PWM');
    grid on;

    %Car Angle
    subplot(2, 1, 2);
    xlabel('Time [sec]')
    yyaxis left
    plot(time_stamp_sec, pitch_rad, 'LineWidth', 1.3)
    ylabel('Car Angle [rad]')
    yyaxis right
    plot(time_stamp_sec, command_pwm, 'LineWidth', 1.5);
    ylabel('Command PWM');
    grid on;
end

%% Motor Characterizing

for test_num = 1:num_tests
    parse = arduino_parse(test_num);

    step_pwm = parse.test_info(1).value;
    dt_us = parse.test_info(2).value;
    time_to_step_ms = parse.test_info(3).value;
    step_duration_ms = parse.test_info(4).value;

    time_stamp_sec = (parse.data(:, 1) - parse.data(1, 1)) / 1000000;%normalize to 0, scale to seconds
    wheel_speed_rad_sec = parse.data(:, 2);
    command_pwm = parse.data(:, 3);
   
    
    step_ndx = command_pwm ~= 0;
    if (~any(step_ndx))
        continue;
    end
    step_resp_rad_s = wheel_speed_rad_sec(step_ndx);
    step_time = time_stamp_sec(step_ndx);
    step_time = step_time - step_time(1);

    %Build a model (simple)
    [~, top_ndx] = max(step_resp_rad_s);
    ss_ndx = top_ndx:length(step_resp_rad_s); %index of steady state

    w_ss = mean(step_resp_rad_s(ss_ndx)); %Kp * Km / (1+Kp*Km)
    Km_w_pwm = w_ss / step_pwm;  %put km in units of rad/sec/pwm

    taus_to_check = [0.3, 0.5, 0.75, 1, 1.5, 1.7, 2, 2.5, 3]; 
    num_taus_check = length(taus_to_check);
    found_tau_ndx = zeros(1, num_taus_check); 
    for tau_ndx = 1:num_taus_check   
        checking_tau = taus_to_check(tau_ndx);
        scale_of_ss = 1 - exp(-checking_tau);
        found_ndx = find(abs(w_ss * scale_of_ss) <= abs(step_resp_rad_s), 1, "first"); %find first point where we are past the tau
        found_tau_ndx(tau_ndx) = found_ndx;
    end
    found_tau_times = time_stamp_sec(found_tau_ndx);
    tau_estimates = found_tau_times' ./ taus_to_check;%scale the time down to # of taus
    effective_tau = mean(tau_estimates);

    motor_model = tf([Km_w_pwm], [effective_tau, 1]);


    figure('Name', sprintf('PWM: %g Step Test', step_pwm));

    %Wheel Speed
    xlabel('Time [sec]')
    plot(step_time, step_resp_rad_s, 'LineWidth', 1.3);
    ylabel('Wheel Speed [rad/sec]')
    hold on
    model_step = stepplot(step_pwm * motor_model);
    model_step.Responses(end).LineWidth = 1.6;
    model_step.Responses(end).Color = [1, 0, 0];
    hold off
    legend('Collected Data', sprintf('Model: Km = %g rad/sec/pwm, tau = %g sec', Km_w_pwm, effective_tau))
    grid on;


    %Should I be trying to grab the dampning from the fall off?
end
