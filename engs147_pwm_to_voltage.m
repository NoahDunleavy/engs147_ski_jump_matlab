%Noah Dunleavy
%ENGS147 - Ski Jump PWM to Current
clc; clear; close all;

%% Data Collection and Params

%Around 11.1V

%Deadzone PWM: [-3, 3]
%PWM Range: [-255, 255]

pwm_voltage_list = [
   %Negatve PWMS
   -255, 11.00;
   -250, 10.78;
   -200, 8.56;
   -150, 6.35;
   -100, 4.20;
   -50, 2.09;
   -25, 1.05;
   -10, 0.42;
   -5, 0.20;
   -4, 0.16;
   -3, 0.11; %db
   -2, 0.07; %db
   -1, 0.03; %db
   %Positive PWMS
   1, -0.03; %db
   2, -0.07; %db
   3, -0.11; %db
   4, -0.16;
   5, -0.20;
   10, -0.42;
   25, -1.05;
   50, -2.05;
   100, -4.18;
   150, -6.33;
   200, -8.58;
   250, -10.77;
   255, -10.99
];

save pwm_voltage_list.mat pwm_voltage_list

%% Curve Generation
collected_volt = pwm_voltage_list(:, 2);
collected_pwm = pwm_voltage_list(:, 1);
interp_volt = collected_volt; %for now, just the data points
interp_pwm = interp1(collected_volt, collected_pwm, interp_volt, 'spline', 'extrap'); %use a spline fit for non-linearity, and extrapolate outside of given data


figure('Name', 'Interpolated PWM vs Current')
raw_plot = plot(collected_volt, collected_pwm, 'LineWidth', 2.4, 'Color', [0.3922    0.8314    0.0745]);
ylabel('Applied PWM', 'FontSize', 20)
xlabel('Voltage across motor [A]', 'FontSize', 20)
title('Interpolated PWM vs Voltage', 'FontSize', 32)
xlim([-10, 10])
grid on
hold on;
interp_plot = plot(interp_volt, interp_pwm, 'LineWidth', 2, 'Color', [0.9294    0.6941    0.1255]);
hold off;
temp_leg = legend('Collected Data', 'Matlab Model');
set(temp_leg, 'FontSize', 24);


%% Create Arduino File

%needs to be sorted by lowest to highest voltage
[interp_volt, sortIdx] = sort(interp_volt,'ascend');
interp_pwm = interp_pwm(sortIdx) 

create_cpp_LUT("C:\Users\noahd\OneDrive\Documents\Arduino\libraries\ENGS147_Ski_Jump\pwm_lookup.h", interp_volt, interp_pwm, 0.11, 3); %create/update LUT

