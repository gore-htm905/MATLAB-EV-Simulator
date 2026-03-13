clc;
clear;
close all;

%% PARAMETERS

m = 1500;              % Vehicle mass (kg)
Cd = 0.29;             % Drag coefficient
A = 2.2;               % Frontal area (m^2)
rho = 1.2;             % Air density (kg/m^3)
Cr = 0.015;            % Rolling resistance coefficient
g = 9.81;              % Gravity (m/s^2)

Rw = 0.3;              % Wheel radius (m)
G = 9;                 % Gear ratio

V_batt = 300;          % Battery voltage (V)
Capacity_kWh = 30;     
Q = (Capacity_kWh*1000*3600)/V_batt;  % Battery capacity in Coulomb

SOC = 1;               % Initial SOC (100%)

%% SIMULATION SETTINGS

dt = 0.1;
t = 0:dt:880;          % 880 sec drive cycle
n = length(t);

v = zeros(1,n);        % Vehicle speed
a = zeros(1,n);        % Acceleration
I = zeros(1,n);        % Battery current
SOC_vec = zeros(1,n);  % SOC tracking
distance = zeros(1,n);

%% SIMPLE SPEED REFERENCE (example NEDC-like)

v_ref = 15 + 10*sin(0.01*t);

%% SIMULATION LOOP

for k = 2:n
    
    % Speed error
    error = v_ref(k-1) - v(k-1);
    
    % Simple proportional controller
    Kp = 500;
    F_trac = Kp * error;
    
    % Limit tractive force
    F_trac = max(min(F_trac, 4000), -3000);
    
    % Resistive forces
    F_drag = 0.5*rho*A*Cd*v(k-1)^2;
    F_roll = m*g*Cr;
    
    % Net force
    F_net = F_trac - F_drag - F_roll;
    
    % Acceleration
    a(k) = F_net/m;
    
    % Update speed
    v(k) = v(k-1) + a(k)*dt;
    
    % Prevent negative speed
    if v(k) < 0
        v(k) = 0;
    end
    
    % Motor torque
    T_motor = (F_trac * Rw) / G;
    
    % Power calculation
    P = F_trac * v(k);
    
    % Battery current
    I(k) = P / V_batt;
    
    % SOC update
    SOC = SOC - (I(k)*dt)/Q;
    SOC_vec(k) = SOC;
    
    % Distance
    distance(k) = distance(k-1) + v(k)*dt;
    
end

%% PLOTS

figure;
plot(t, v, 'b', t, v_ref, 'r--','LineWidth',1.5);
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend('Vehicle Speed','Reference Speed');
title('Speed Tracking');

figure;
plot(t, SOC_vec,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('State of Charge');
title('Battery SOC');

figure;
plot(t, distance/1000,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Distance (km)');
title('Distance Travelled');
