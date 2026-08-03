
% PMSM Parameters
Rs  = 2.875;        % Stator resistance (Ohm)
Ld  = 6e-3;         % d-axis inductance (H)
Lq  = 10.5e-3;      % q-axis inductance (H)
lam = 0.175;        % PM flux linkage (W)
P   = 4;            % Pole pairs
J   = 0.001;        % Rotor inertia (kg.m^2)
B   = 0.05; 
Ron = 0.01;         % Viscous damping (N.m.s/rad)

% 3 phase Inverter Parameters
R_mosfet=0.0175;    % Mosfet resistance (ohms)
V_threshold=4;      % threshold voltage (V)
V_forward=0.9;      % forward voltage (V)
V_dc = 48;          % DC bus voltage (V)
f_sw = 20000;       % Switching frequency (Hz)
Ts   = 50e-6;       % Simulation timestep (s)

% FOC Parameters
id_ref = 0;         % d-axis reference (A)
iq_ref = 0.5;       % q-axis reference (A)

% PI Gains
Kp_id = 37.7;       % id proportional gain
Ki_id = 18063;      % id integral gain
Kp_iq = 65.97;      % iq proportional gain
Ki_iq = 18063;      % iq integral gain

% Limits
V_lim = 27.7;       % voltage saturation limit (~27.7V)
% Speed Loop Parameters
speed_ref_value = 10;   % rad/s (your target speed)
Kp_speed = 0.05;           % start small
Ki_speed = 0.5;            % start small
iq_max = 5;                % maximum current allowed (Amps) - safety limit

% MOSFET/Switch datasheet-typical values
C_oss = 100e-12;         % Output capacitance (F) - typical small low-voltage MOSFET

% DC bus ripple tolerance
delta_Vdc_percent = 0.02;           % 2% ripple allowed (per road map spec)
delta_Vdc = delta_Vdc_percent * V_dc;

% Rated current (choose your tested, stable working value)
iq_rated = 0.5;         % Amps - UPDATE this to match your last successful stable test value



