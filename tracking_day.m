function metrics = tracking_day()
%TRACKING_DAY Real-time accuracy & energy of the tracker servo.
%
%   Full 12 h daylight window (06:00-18:00 solar) in real time. Drive:
%   voltage-mode PID + gravity feedforward; hardware motor-speed clamp
%   (structural slew limit); implicit armature current; one-way
%   (self-locking worm) gearbox with backlash and shaft compliance.
%
%   Design summary (the deployable controller):
%   - continuity is free with a worm gear: zero holding current;
%   - a 0.1 deg deadband eliminates drive dither (hunting through the
%     backlash gap at 50 Hz would otherwise dominate the wear metric);
%   - gravity is compensated CONTINUOUSLY (tanh ramp), not with a sign
%     flip, so no chatter around the setpoint;
%   - the motor current is integrated implicitly (stiff L/R pole).

% Gains and band
Kp    = 90;   Ki = 4;   Kd = 5;        % V/rad, V/(rad*s), V/(rad/s)
dbDeg = 0.10;                          % position deadband [deg]
kappa = 0.15;                          % gravity-ramp smoothness [1/V]
wmaxP = deg2rad(1.75);                 % structural max panel slew [rad/s]

p   = servo_params();
lat = 34; doy = 355;

dtc  = 0.020;                    % controller period [s]
span = 12*3600;
T    = 0:dtc:span;
tSun = 6 + T/3600;

% ---- reference: panel tilt = solar elevation ----
[alpha, ~, ~] = sun_position(lat, doy, tSun);
ref = min(max(rad2deg(alpha), 0), 85);

% ---- wind disturbance ----
kw = 0.5*1.225*1.2*2*0.75;
v  = 5.5 + 2.2*sin(2*pi*T/7200 + 0.7) + 1.5*exp(-mod(T + 10, 500)/60);
Tw = kw*v.^2;

% ---- controller ----
KV    = p.Kt/p.R*p.N*p.eta;      % voltage -> panel torque [N*m/V]
wmaxM = wmaxP*p.N;               % equivalent motor rad/s clamp

% ---- plant (scalar closures) ----
N = p.N; Jm = p.Jm; Jl = p.Jl; bm = p.bm; Tcm = p.Tcm;
blms = p.blms; Tcp = p.Tcp; Kt = p.Kt; Ke = p.Ke; R = p.R;
Vmax = p.Vmax; ks = p.ks; cs = p.cs; bl = p.bl;
m = p.m; r = p.r; g = p.g; taui = p.L/p.R;

% initialise the shaft in gravity equilibrium so the panel holds its
% pose from t=0 (spring pre-wound by the gravity load); otherwise the
% controller must first recover from an unloaded initial state.
blReach = p.bl + (p.m*p.g*p.r/p.ks);   % backlash + gravity deflection
thm = blReach*p.N;                     % motor angle = pre-wound load hold
wm = 0; thp = 0; wp = 0; iArm = 0; es = 0;
n   = numel(T);
errDeg = zeros(1,n); thPs = zeros(1,n); iArms = zeros(1,n); E = 0;

% Latching deadband: once the drive settles inside the band it stays locked
% until the error exceeds a wider exit threshold, which is what actually
% prevents chatter -- the self-locking worm gear does not release the panel
% once it is stopped, so re-engaging on every tiny excursion would just
% restart hunting through the backlash gap.
dbEnter = deg2rad(dbDeg);
dbExit  = deg2rad(dbDeg)*2.0;
latched = false;

for k = 2:n
    e = deg2rad(ref(k)) - thp;

    % Latching deadband (see note above)
    if latched
        if abs(e) > dbExit
            latched = false;
        end
    elseif abs(e) <= dbEnter
        latched = true;
    end

    % Deadband: inside the band the self-locking worm gear holds the panel
    % at zero current and the drive stops hunting through the backlash.
    if latched
        uc = 0;  es = 0;
    else
        % PID without feedforward first
        u   = Kp*e - Kd*wp;
        uS  = max(-Vmax, min(Vmax, u));
        if abs(u) <= Vmax
            es = es + Ki*e*dtc;
        else
            es = es + Ki*e*dtc + (uS - u)*4*dtc;   % unwind while saturated
        end
        uc0 = max(-Vmax, min(Vmax, u + es));

        % Continuous gravity compensation. A hard sign() flip would make
        % the drive hunt around the setpoint (gravity FF jumps +/-20 V as
        % the command crosses zero). tanh() ramps 0 -> full weight
        % smoothly, so the drive only takes the weight as command grows.
        G = m*g*r*cos(thp)/KV;             % full-weight hold voltage
        uc = uc0 + G*tanh(kappa*uc0);
        uc = max(-Vmax, min(Vmax, uc));
    end

    % ---- inner integration, 4 substeps per controller period ----
    for ii = 1:4
        h = dtc/4;
        wm = max(-wmaxM, min(wmaxM, wm));          % structural RPM clamp
        iArm = (taui*iArm + (uc - Ke*wm)*h/R)/(taui + h); % implicit current (L/R pole)
        Tm = Kt*iArm;

        dth = thm/N - thp;
        if abs(dth) < bl
            Ts = 0;
        else
            dth2 = dth - sign(dth)*bl;
            Ts   = ks*dth2 + cs*(wm/N - wp);
        end

        % motor: self-locking worm gear does not transmit load back
        dwm = (Tm - bm*wm - Tcm*sign(wm))/Jm;
        wm  = max(-wmaxM, min(wmaxM, wm + dwm*h));

        % panel with gravity + wind load (wind is a disturbance, not friction)
        Twind = Tw(k)*cos(thp);          % wind torque has its own sign
        gT    = m*g*r*cos(thp);
        dwp   = (Ts - blms*wp - Tcp*sign(wp) - Twind - gT)/Jl;
        wp    = wp + dwp*h;
        thp   = thp + wp*h;  thm = thm + wm*h;

        E = E + uc*iArm*h;                           % J (per 2 m2)
    end

    errDeg(k) = rad2deg(e);
    thPs(k)   = thp;
    iArms(k)  = iArm;
end

E = E/3600/2;                                    % Wh/m2

fprintf('PID tracker, real time, worm-gear drive (deadband %.2f deg):\n', dbDeg);
fprintf('  RMS tracking error   : %.4f deg\n', rms(errDeg));
[~,im] = max(abs(errDeg));
fprintf('  Peak error           : %.4f deg  (%.1f h solar, sun at horizon)\n', ...
        abs(errDeg(im)), 6 + T(im)/3600);
fprintf('  Control energy       : %.4f Wh/m2/day\n', E);
fprintf('  Panel motion (total) : %.0f deg\n', rad2deg(sum(abs(diff(thPs)))));
fprintf('  Peak motor current   : %.1f A (stall %.0f A)\n', max(abs(iArms)), p.Vmax/p.R);

figure('Name','Real-time tracker','Position',[80 100 1000 620]);
subplot(3,1,1);
plot(T/3600+6, ref, 'k--', T/3600+6, rad2deg(thPs), 'b', 'LineWidth',1.1); grid on;
legend({'Solar elevation','Panel (PID)'},'Location','northwest');
ylabel('Angle [deg]'); xlabel('Solar hour');
title('Real-time PID tracking, winter day, gusty wind (worm-gear drive)');
subplot(3,1,2);
plot(T/3600+6, errDeg); grid on;
ylabel('Tracking error [deg]'); xlabel('Solar hour');
subplot(3,1,3);
yyaxis left; plot(T/3600+6, Tw.*cos(thPs)); ylabel('Wind torque [N*m]');
yyaxis right; plot(T/3600+6, v, 'r:'); ylabel('Wind speed [m/s]');
legend({'Wind torque','Wind speed'},'Location','northwest'); xlabel('Solar hour');

metrics = struct('rmsDeg',rms(errDeg),'peakDeg',max(abs(errDeg)),...
                 'whPerM2',E,'motionDeg',rad2deg(sum(abs(diff(thPs)))),...
                 'peakCurrentA',max(abs(iArms)));
end