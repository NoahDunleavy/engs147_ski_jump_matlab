%ENGS147 - Car Model Updating
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

%Updated Specs (from -4.6V step input, 63 rad/sec)
K_b = 0.00604325;
J_rotor = 6.00276e-5;

torque_to_shaft = tf(1, [J_rotor, 0]);
gear_box = tf(1, gear_ratio);


%% Build the Model
filepath = "C:\Users\noahd\OneDrive\Desktop\ENGS 147\Final Project\j_car_update_step.txt";
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


    %Do a different tau calculation [TODO]
    max_ndx = find(wheel_speed_rad_sec > 0.95 * Km, 1, 'first');%go untul the 3 tau point
    found_taus = zeros(max_ndx, 1);
    for tau_ndx = 1:max_ndx   
        current_time = time_stamp_sec(tau_ndx);%get the t
        current_speed = wheel_speed_rad_sec(tau_ndx);
        current_tau = -current_time / log(1 - current_speed / Km);
        found_taus(tau_ndx) = current_tau;
    end
    effective_tau = mean(found_taus);

    new_J_body = effective_tau * new_Kb * J_rotor * K_torque / (R_a * J_rotor - effective_tau * K_torque * new_Kb); %[kg*m^2]

    %Check model
    new_torque_to_body = tf(-1, [new_J_body, 0]);
    new_torque_to_relative = torque_to_shaft - new_torque_to_body;
    new_Vin_to_torque = feedback((1/R_a) * K_torque, new_Kb * new_torque_to_relative, -1);
    new_Vin_to_relative = new_Vin_to_torque * new_torque_to_relative;
    new_Vin_to_wheel = new_Vin_to_relative * gear_box;
    [step_rotor, step_time] = step(step_voltage * new_Vin_to_wheel, time_stamp_sec(1):(dt_ms/1000):time_stamp_sec(end));

    hold on;
    plot(step_time, step_rotor, 'LineWidth', 1.2);
    hold off;
    legend('Collected Data', 'New Model')
    subtitle(sprintf('Update Parameters:\nKb=%g\nJ=%g', new_Kb, new_J_wheel));

    Kb_J_v(test_num, :) = [new_Kb, new_J_wheel, step_voltage];
end

sprintf('Kb [V/rad/sec], J [kg * m^2], Vin [V]')
Kb_J_v