%ENGS147 - Model Updating
clc; clear; close all;
%% Starting Parameters
%Motor Specs (per data sheet as of 5/23/25 - https://cdn.andymark.com/media/W1siZiIsIjIwMTgvMTIvMDYvMDkvMTgvNTMvMTBmYTQ5OTItZDQ2Zi00YzA1LWJmYzMtOTVjNDMwMDgwOWY0L2FtLTM3NzVhIFJlZExpbmUgUGVyZm9ybWFuY2UgQ3VydmUuUERGIl1d/am-3775a%20RedLine%20Performance%20Curve.PDF?sha=6a99f4a3d13a7b36)
i_stall = 130;  %[A]
i_no_load = 3.8; %[A]
w_no_load = 21020 * 2*pi / 60; %[rpm -> rad/sec]
torque_stall = 0.7; %[Nm]
V_motor_rate = 12; %[V]
K_torque = 0.01; %[Nm/A]
dynamic_res = 0.092; %[ohm]

%Motor Specs (calculated from specs)
%Motor model:
%    +-------+-------+
%            |       |
%            |      R_a
%   Vin     R_nl     |
%            |     V_efm
%            |       |
%    +-------+-------+-->

R_nl = V_motor_rate / i_no_load;
R_a = V_motor_rate / (i_stall - i_no_load); %how much current is going through the motor when V_emf = 0
K_b = V_motor_rate / w_no_load;


%System Specs
gear_ratio = 12; %[unitless]
I_wheel = 1.534356 * (1 / 254)^2; %[kg*in^2 -> kg*m^2]
I_car = 14.056091 * (1 / 254)^2; %[kg*in^2 -> kg*m^2]

J_rotor = I_wheel; %J = Iw
J_car = I_car;

%% Build the Model
filepath = "C:\Users\noahd\OneDrive\Desktop\ENGS 147\Final Project\j_update_step.txt";
arduino_parse = parse_arduino_log(filepath);
num_tests = length(arduino_parse);

Kb_J_v = zeros(num_tests, 3);

for test_num = 1:num_tests
    parse = arduino_parse(test_num);

    step_voltage = parse.test_info(1).value;
    if step_voltage == 0
        continue
    end
    dt_ms = parse.test_info(2).value;

    time_stamp_sec = (parse.data(:, 1) - parse.data(1, 1)) / 1000000;%normalize to 0, scale to seconds
    wheel_speed_rad_sec = parse.data(:, 2);
    command_volt = parse.data(:, 3);
    command_pwm = parse.data(:, 4);

    figure('Name', sprintf('Step Voltage: %g', step_voltage));
    %Plot Step
    plot(time_stamp_sec, wheel_speed_rad_sec);
    xlabel('Time [sec]')
    ylabel('Wheel Speed [rad/sec]')
    title(sprintf('Step: %g V', step_voltage))

    %Back out model
    ss_ndx = find(time_stamp_sec > 1, 1, 'first');%rough, but for our demos it works
    Km = mean(wheel_speed_rad_sec(ss_ndx:end)) / step_voltage; %[rad/sec/V]

    new_Kb = 1 / (gear_ratio * Km); %[V/rad/sec]

    taus_to_check = [0.25, 1, 1.5]; 
    num_taus_check = length(taus_to_check);
    found_tau_ndx = zeros(1, num_taus_check); 
    for tau_ndx = 1:num_taus_check   
        checking_tau = taus_to_check(tau_ndx);
        scale_of_ss = 1 - exp(-checking_tau);
        found_ndx = find(abs(Km * scale_of_ss) <= abs(wheel_speed_rad_sec), 1, "first"); %find first point where we are past the tau
        found_tau_ndx(tau_ndx) = found_ndx;
    end
    found_tau_times = time_stamp_sec(found_tau_ndx);
    tau_estimates = found_tau_times' ./ taus_to_check;%scale the time down to # of taus
    effective_tau = mean(tau_estimates);

    new_J_wheel = effective_tau * new_Kb * K_torque / R_a; %[kg*m^2]

    %Check model
    motor_model = tf(1, [new_J_wheel, 0]);
    V_in_to_rotor = feedback(1/R_a * K_torque * motor_model, new_Kb, -1);
    [step_rotor, step_time] = step(step_voltage * V_in_to_rotor / gear_ratio, time_stamp_sec(1):(dt_ms/1000):time_stamp_sec(end));

    hold on;
    plot(step_time, step_rotor, 'LineWidth', 1.2);
    hold off;
    legend('Collected Data', 'New Model')
    subtitle(sprintf('Update Parameters:\nKb=%g\nJ=%g', new_Kb, new_J_wheel));

    Kb_J_v(test_num, :) = [new_Kb, new_J_wheel, step_voltage];
end

sprintf('Kb [V/rad/sec], J [kg * m^2], Vin [V]')
Kb_J_v