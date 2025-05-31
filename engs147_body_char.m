%ENGS147 - Ski Jump Body Characterization
clc; clear; close all;
title_size = 60; %font size for plots
label_size = 44;
legend_size = 20;
%Options for plotting
pzopt = pzoptions;
pzopt.Grid = 'on';
pzopt.GridColor = [0.5 0.5 0.5];
pzopt.Title.FontSize = 11;
pzopt.Title.FontWeight = 'bold';
pzopt.XLabel.FontSize = 11;
pzopt.YLabel.FontSize = 11;
s = settings;
s.matlab.appearance.figure.GraphicsTheme.TemporaryValue = "light";

%% System Parameters

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

%% System model
dt = 0.05; %sample period [sec]
pwm_in = 150;
V_in = 12 * pwm_in / 255;

%Continuous blokcs
torque_to_shaft = tf(1, [J_rotor, 0]);
torque_to_body = tf(-1, [J_car, 0]);
torque_to_relative = torque_to_shaft - torque_to_body;
torque_to_angle = torque_to_body * tf(1, [1, 0]);
Vin_to_torque = feedback((1/R_a) * K_torque, K_b * torque_to_relative, -1);
Vin_to_relative = Vin_to_torque * torque_to_relative;

gear_box = tf(1, gear_ratio);
Vin_to_wheel = Vin_to_relative * gear_box;
Vin_to_angle = Vin_to_torque * torque_to_angle;

figure('Name', 'Open Loop Continous Step Wheel Speed')
stepplot(V_in * Vin_to_wheel);

figure('Name', 'Open Loop Continous Step Body Angle')
stepplot(V_in * Vin_to_angle);

Gz_to_wheel = c2d(Vin_to_wheel, dt, 'zoh'); %discrete model of plant
Gz_to_angle = c2d(Vin_to_angle, dt, 'zoh');

%% Root Loucs
figure('Name', 'Uncompensated Discrete Open Loop Root Locus')
rl_disc_ol = rlocusplot(Gz_to_wheel, pzopt);
rl_disc_ol.Responses.MarkerSize = 10;
rl_disc_ol.Responses.LineWidth = 1.6;
title('\bfDiscrete Uncompensated Open-Loop Root Locus');
subtitle(sprintf('dt = %g sec', dt));

%% Compensator Design

%v1
Gcz = zpk([0.4, 0.53], [0.35, 1], 1, dt);
K = 0.16;
Hs = 1;
wheel_ref = 7.5; 

%Open Loop for K selection
disc_open_loop = Gcz * Gz_to_wheel;
figure('Name', 'Discrete Open Loop Root Locus')
rl_disc_comp_ol = rlocusplot(disc_open_loop, pzopt);
rl_disc_comp_ol.Responses.MarkerSize = 10;
rl_disc_comp_ol.Responses.LineWidth = 1.6;
title('\bfDiscrete Root Locus');
subtitle(sprintf('dt = %g sec', dt));

%Closed Loop behavior
disc_cl = feedback(K*Gcz*Gz_to_wheel, Hs, -1);

[wn,zeta,p] = damp(disc_cl); %compute freq, dampning, and poles
hold on;
plot(real(p),imag(p),'x','MarkerSize',10,'LineWidth',2,'Color',[0.8 0 0], 'DisplayName', 'Closed-loop Poles');
hold off;

%Difference equation
[comp_num, comp_denom] = tfdata(K*Gcz, 'v');
comp_num = comp_num / comp_denom(1);
comp_denom = comp_denom / comp_denom(1);
fprintf('\n\nv(k) = '); %kick of the first u(k)
if (length(comp_denom) > 1)
    for ndx = 2:length(comp_denom) 
        fprintf('%+.4g*v(k-%d) ', -comp_denom(ndx), ndx-1);
    end
end
    fprintf('%+.4g*e(k) ', comp_num(1));
if (length(comp_num) > 1)
    for ndx = 2:length(comp_num)
        fprintf('%+.4g*e(k-%d) ', comp_num(ndx), ndx-1);
    end
end
fprintf('\n');


[wheel_speed, step_time] = step(wheel_ref*disc_cl);
%Compute the theoretical input/current
step_input_volt = zeros(size(wheel_speed));
for ndx = 1:length(step_input_volt)
    if ndx <= 1%if we are first step
        U1 = 0;
        E1 = 0;
        E2 = 0;
        U2 = 0;
    elseif ndx <= 2
        U1 = step_input_volt(ndx - 1);
        E1 = wheel_ref - wheel_speed(ndx - 1);
        E2 = 0;
        U2 = 0;
    else
        U1 = step_input_volt(ndx - 1);
        E1 = wheel_ref - wheel_speed(ndx - 1);
        U2 = step_input_volt(ndx - 2);
        E2 = wheel_ref - wheel_speed(ndx - 2);
    end
    E0 = wheel_ref - wheel_speed(ndx);

    step_input_volt(ndx) = -comp_denom(2)*U1 -comp_denom(3)*U2 + comp_num(1)*E0 + comp_num(2)*E1 +comp_num(3)*E2;
    %step_input_volt(ndx) = K * E0;
end
step_Va = step_input_volt - wheel_speed*gear_ratio*K_b;


% plot results
figure('Name', 'Panelled Direct Discrete Design');
% set(gcf,'Position',[0152 0362 1473 0426]);
tl2 = tiledlayout(1,3);
nexttile(1);
rp1 = rlocusplot(Gcz*Gz_to_wheel,pzopt); %
rp1.Responses.LineWidth = 1.6;
rp1.Responses.MarkerSize = 10;
hold on;
plot(real(p),imag(p),'x','MarkerSize',10,'LineWidth',2,'Color',[0.8 0 0]);

% adjust scaling of root locus for easy viewing
xlim([-1.05 1.05]);
ylim([-1.05 1.05]);

ax2(1) = nexttile(2);
stp5 = stepplot(wheel_ref * disc_cl);
stp5.Responses.LineWidth = 1.6;
grid on;
hold on;
title('\bfOutput Step');
subtitle(sprintf('dt = %g sec', dt));
ylabel('Wheel Speed [rad/sec]')

ax2(2) = nexttile(3);
stp6 = stairs(step_time, step_Va/R_a + step_input_volt/R_nl);
grid on;
title('\bfCurrent Control Step');
subtitle(sprintf('dt = %g sec', dt));
ylabel('Amps [A]')

linkaxes(ax2,'x');

return