function manual_servo()
%MANUAL_SERVO Validate the elevation-axis servo and tune the PID
%   controller in plain MATLAB before mirroring the model in Simulink.
%
%   Reference: sun-elevation profile from sun_position() on a gusty
%   overcast day, in "demo time" (the whole day condensed into 12 s of
%   simulation time). The PID drives motor voltage; the plant includes
%   backlash, torsional compliance, gravity and wind loads.
%   Metrics: RMS/peak tracking error, control energy, total slew.

clc; close all;
p = servo_params();

% ---- demo axes: compress 12 h (6 h..18 h) into 12 s ----
tDay = 6:0.002:18;                 % solar hours at 2 ms of demo per hour
tsim = (tDay - 6);                 % demo seconds: 0..12
t0   = tsim;                       % demo time axis

% ---- sun elevation reference for a winter day (low path, demanding) ----
lat  = 34; doy = 355;
[alpha, ~, ~] = sun_position(lat, doy, tDay);
refDeg = rad2deg(alpha);            % panel tilt command = solar elevation
refDeg = min(refDeg, 85);
refDeg = max(refDeg, 0);

% ---- wind torque disturbance (gusty day) ----
v   = gen_wind('gusty', tDay, 3);
kw  = 0.5*1.225*1.2*2*0.75;         % T_wind = kw*v^2   [N*m]
Tw  = kw*v.^2;                      % on tilt axis (own sign = disturbance)

% ---- gravity torque to feed forward ----
Tgrav = p.m*p.g*p.r;                % peak gravity torque at beta=0

% ---- controller gains (hand-tuned from the linearisation) ----
KV = p.Kt/p.R*p.N*p.eta;            % voltage -> panel-axis torque [N*m/V]
Kp  = 90.0;      % V/rad
Ki  = 4.0;       % V/(rad*s)
Kd  = 5.0;       % V/(rad/s)

% ---- plant state [theta_m, omega_m, theta_p, omega_p] ----
th_m = 0; w_m = 0; th_p = 0; w_p = 0;
es = 0;                             % integral state (anti-windup later)

n   = numel(t0);
thP = zeros(1,n); wP = zeros(1,n);
uV  = zeros(1,n); uI = zeros(1,n); uP = zeros(1,n); uD = zeros(1,n);
Pdc = zeros(1,n);

for k = 2:n
    dtk = t0(k) - t0(k-1);

    % ---- PID on panel angle (rad) ----
    ref = deg2rad(refDeg(k));
    e   = ref - th_p;
    uP(k)   = Kp*e;
    uD(k)   = -Kd*(w_p);            % derivative on measured angle only
    % gravity feedforward: cancel m*g*r*cos(theta) at the panel
    uFF     = (p.m*p.g*p.r*cos(th_p))/KV;
    % integral with back-calculation anti-windup
    u       = Kp*e + uD(k) + uFF;
    uSat    = max(-p.Vmax, min(p.Vmax, u));
    if abs(u) <= p.Vmax
        es = es + Ki*e*dtk;
    else
        es = es + Ki*e*dtk + (uSat - u)*4*dtk;   % unwind while saturated
    end
    uI(k) = es;
    uo    = max(-p.Vmax, min(p.Vmax, u + es));   % final clamped output
    uV(k) = uo;
    u     = uo;

    % ---- motor electrical ----
    I = (u - p.Ke*w_m)/p.R;         % armature current (L neglected)
    Tm = p.Kt*I;

    % ---- gearbox: backlash + compliance on panel side ----
    th_gs = th_m/p.N;               % gearbox output angle
    dth   = th_gs - th_p;           % spring deflection at panel axis
    if abs(dth) < p.bl
        Ts = 0;                     % in backlash dead zone
    else
        dth  = dth - sign(dth)*p.bl;
        Ts   = p.ks*dth + p.cs*(w_m/p.N - w_p);   % spring+damping torque
    end

    % ---- panel axis dynamics ----
    Twind = Tw(k) * cos(th_p);      % wind torque has its own sign (disturbance)
    gT = Tgrav*cos(th_p);           % gravity torque (worst at beta=0)
    dw_p = ( Ts - p.blms*w_p - sign(w_p)*p.Tcp - Twind - gT ) / p.Jl;

    % ---- motor axis dynamics (gear reaction + friction) ----
    Treact = Ts / (p.N*p.eta);
    dw_m  = ( Tm - p.bm*w_m - sign(w_m)*p.Tcm - Treact ) / p.Jm;

    % ---- integrate (forward Euler) ----
    w_p = w_p + dw_p*dtk;   th_p = th_p + w_p*dtk;
    w_m = w_m + dw_m*dtk;   th_m = th_m + w_m*dtk;

    thP(k)=th_p; wP(k)=w_p;
    Pdc(k)= u*I;                 % DC power drawn
end

% ---- metrics ----
ref = deg2rad(refDeg);
err = ref - thP;
rms_deg = rad2deg(rms(err));
pk_deg  = rad2deg(max(abs(err)));
% settle after first 5% of trace (skip initial sunrise transient)
E = trapz(t0, Pdc)/3600;                 % Wh
motion = sum(abs(diff(thP)));            % rad of load motion

fprintf('PID Kp=%.0f Ki=%.1f Kd=%.1f\n', Kp, Ki, Kd);
fprintf('RMS tracking error  : %.3f deg\n', rms_deg);
fprintf('Peak error          : %.3f deg\n', pk_deg);
fprintf('Control energy      : %.2f Wh (=> per m2: %.2f Wh/m2)\n', E, E/2);
fprintf('Total panel motion  : %.0f deg\n', rad2deg(motion));

figure('Name','Servo validation','Position',[80 100 1100 650]);
subplot(3,1,1);
plot(t0, refDeg, 'k--', t0, rad2deg(thP), 'b', 'LineWidth',1.2); grid on;
legend('Reference','Panel (PID)','Location','best'); ylabel('Elevation [deg]');
title('Angle servo');
subplot(3,1,2);
plot(t0, rad2deg(err)); grid on; ylabel('Error [deg]');
subplot(3,1,3);
plot(t0, uV); grid on; ylabel('Motor voltage [V]'); xlabel('Demo time [s]');
end