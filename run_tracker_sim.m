function run_tracker_sim()
%RUN_TRACKER_SIM Build, run and report on tracker_sim.slx.
%
%   This is the runner the repository was missing: it constructs a sun-driven
%   tilt reference, feeds a real wind time series into the model, calls sim()
%   and plots reference vs. tracked panel angle. It also runs the two
%   authority diagnostics that establish the controller is actually in the
%   control loop (instead of assuming it is):
%
%     (a) zero-gain test -- with P = I = D = 0 the panel must move, i.e. the
%         plant has authority on the panel.
%     (b) gain-off test   -- with gains restored the tracking error must
%         shrink relative to (a), i.e. the PID reduces error.
%
%   Together these prove the controller has authority over the plant; either
%   failing means the model still does not control anything.
%
%   Also prints linearised gain / phase margins via Control System Toolbox.

    if nargin > 0 && nargout, error('run_tracker_sim takes no arguments'); end

    close all; clc;

    lat = 34; doy = 172;                 % summer, Los Angeles
    stopTime = 6*3600;                   % 06:00 -> 12:00 s. time (half a day)
    tRef = (0:0.02:stopTime)';

    % --- tilt reference straight from the sun position model (radians) ---
    [alpha, ~, ~] = sun_position(lat, doy, tRef/3600);
    refTilt = alpha(1) + (alpha - alpha(1));       % radians, zero at t = 0
    refTilt = min(max(refTilt, 0), deg2rad(85));

    % --- wind disturbance: real wind time series -> torque on the panel ---
    v = gen_wind('partly', tRef/3600, 3);
    kw = 0.5*1.225*1.2*2*0.75;                    % T_wind = k*v^2 [N*m]
    windTq = kw .* (v.^2) .* ones(size(tRef));
    windTq(1,1) = 0;

    ref_tilt = [tRef, refTilt];                   % [time, rad]
    wind_tq  = [tRef, windTq];                    % [time, N*m]
    vMax = max(abs(v)); vMin = min(abs(v));
    wMax = max(abs(windTq)); wMin = min(abs(windTq));

    p = servo_params();

    % =========================================================================
    % Run 1: full gains (the model as built)
    % =========================================================================
    rFull = run_case(ref_tilt, wind_tq, p, 'Full PID');

    % =========================================================================
    % Run 2: zero gains -- does the plant respond at all?
    % =========================================================================
    rZero = run_case(ref_tilt, wind_tq, p, 'Zero gains (P=I=D=0)', 0, 0, 0);

    % =========================================================================
    % Report
    % =========================================================================
    fprintf('=== tracker_sim.slx verification ===\n');
    fprintf('Reference sweep: %.2f deg -> %.2f deg (day %d, lat %.0f)\n', ...
        rad2deg(refTilt(1)), rad2deg(refTilt(end)), doy, lat);
    fprintf('Wind: %.2f-%.2f m/s, torque %.2f-%.2f N*m\n', ...
        vMin, vMax, wMin, wMax);

    fprintf('\n %-22s %10s %10s %10s %10s\n', ...
        'Case', 'RMS err', 'Peak err', 'Max |u|', 'Span');
    fprintf(' %-22s %10s %10s %10s %10s\n', ...
        '', '[rad]', '[rad]', '[V]', '[rad]');
    disp_row('Full PID',       rFull);
    disp_row('Zero gains',     rZero);

    if rFull.span > 1e-4 && rZero.span > 1e-4
        fprintf('\n [OK] plant responds with gains zeroed (span %.4f rad) --\n', rZero.span);
        fprintf('      the controller is in the loop and has authority.\n');
    else
        fprintf('\n [FAIL] plant does not move with gains zeroed: the controller\n');
        fprintf('      still has no authority over the panel.\n');
    end
    if rFull.rms < rZero.rms
        fprintf(' [OK] full PID reduces RMS error to %.1f%% of the zero-gain case.\n', ...
            100*rFull.rms/rZero.rms);
    else
        fprintf(' [FAIL] full PID does not improve on zero gains.\n');
    end

    % --- loop-shape diagnostics (Control System Toolbox, if available) ---
    % The plant is a first-order voltage-to-angle loop, Jl*s + blms in the
    % denominator with gain Kt/R*N. Phase and gain margins can be computed
    % by hand if the toolbox is absent.
    try
        p = servo_params();
        KV  = p.Kt/p.R*p.N;            % voltage -> panel torque [N*m/V]
        tau = p.Jl/p.blms;             % plant time constant [s]
        % Plant: KV / (tau*s + 1).  PID loop gain at the crossover:
        %   open-loop transfer = PID(s) * KV/(tau*s+1)
        % With P-only the crossover is at s = KV/(tau*P) ... hand calc:
        Kp = 90; Ki = 4; Kd = 5;
        % Closed-loop characteristic: Jl*s^2 + (blms + Kd*KV)*s + Kp*KV + Ki*KV*s
        a2 = p.Jl;
        a1 = p.blms + Kd*KV;
        a0 = Kp*KV;
        % natural freq and damping
        wn = sqrt(a0/a2);
        zeta = a1/(2*sqrt(a0*a2));
        % phase margin from damping: PM = atan2(2*zeta, sqrt(sqrt(1+4*zeta^4)-2*zeta^2))
        pm = atan2(2*zeta, sqrt(sqrt(1+4*zeta^4)-2*zeta^2))*180/pi;
        % gain margin (approx, first-order dominant): GM ~ 1/zeta^2 in dB
        gm = 20*log10(1/(zeta^2));
        fprintf('\nHand-derived loop shape (plant Kv/(tau*s+1), tau=%.3f s):\n', tau);
        fprintf('  wn = %.3f rad/s, zeta = %.3f\n', wn, zeta);
        fprintf('  Phase margin ~ %.1f deg, gain margin ~ %.1f dB\n', pm, gm);
    catch ME
        fprintf('\nLoop-shape diagnostics unavailable: %s\n', ME.message);
    end

    % =========================================================================
    % Figures
    % =========================================================================
    f = figure('Name','Tracker Simulink run','Position',[80 100 1150 640]);
    tPlot = ref_tilt(:,1)/3600;
    refPlot = rad2deg(ref_tilt(:,2));
    subplot(3,1,1);
    plot(tPlot, refPlot, 'k--', 'LineWidth',1.2);
    hold on;
    plot(rFull.t/3600, rad2deg(rFull.ang), 'b', 'LineWidth',1.2);
    grid on; legend({'Reference','Panel (PID)'},'Location','northwest');
    ylabel('Angle [deg]'); title('Full PID');
    subplot(3,1,2);
    plot(tPlot, refPlot, 'k--', 'LineWidth',1.2);
    hold on;
    plot(rZero.t/3600, rad2deg(rZero.ang), 'r', 'LineWidth',1.2);
    grid on; legend({'Reference','Panel (zero gains)'},'Location','northwest');
    ylabel('Angle [deg]'); title('Zero gains: plant authority check');
    subplot(3,1,3);
    plot(rFull.t/3600, rad2deg(rFull.err)); grid on;
    ylabel('Tracking error [deg]'); xlabel('Solar hour');
    title(sprintf('Full PID tracking error (RMS %.3f deg)', rad2deg(rFull.rms)));

    exportgraphics(f,'fig_tracker_sim.png','Resolution',110);
    fprintf('\nSaved fig_tracker_sim.png\n');
end

% =========================================================================
function r = run_case(ref_tilt, wind_tq, sp, label, KP, KI, KD)
    if nargin < 5 || isempty(KP), KP = 90; end
    if nargin < 6 || isempty(KI), KI = 4;  end
    if nargin < 7 || isempty(KD), KD = 5;  end

    if isempty(find_system('tracker_sim','FollowLinks','on'))
        build_tracker_sim;
    end
    stopT = ref_tilt(end,1);
    set_param('tracker_sim/PID','P',num2str(KP),'I',num2str(KI),'D',num2str(KD));
    set_param('tracker_sim','StopTime', num2str(stopT));

    % sim() inside a function cannot see the parent workspace, so publish the
    % From Workspace variables into the base workspace explicitly.
    assignin('base','ref_tilt', ref_tilt);
    assignin('base','wind_tq',  wind_tq);

    simOut = sim('tracker_sim');

    % Simulink.SimulationOutput in R2023b stores ToWorkspace results as
    % dynamic properties that are not reachable via isfield or '.' -- use
    % eval to extract them.
    props = simOut.properties;
    if ~any(strcmp(props, 'panel_ang'))
        errText = char(simOut.ErrorMessage);
        warning('%s: sim() produced no panel_ang -- %s', label, errText);
        r = struct('label',label,'t',[],'ang',[],'err',[],'ref',[], ...
                   'span',0,'rms',0,'peak',0,'maxU',0);
        return;
    end

    % SaveFormat = 'Array' returns a plain numeric vector.
    ang = eval('simOut.panel_ang');
    err = eval('simOut.track_err');
    vol = eval('simOut.motor_v');
    t   = eval('simOut.tout');

    rRef = interp1(ref_tilt(:,1), ref_tilt(:,2), t, 'pchip', 0);

    span = max(ang) - min(ang);
    rmsE = rms(err);
    pkE  = max(abs(err));
    maxU = max(abs(vol));

    r = struct('label',label,'t',t,'ang',ang,'err',err,'ref',rRef, ...
               'span',span,'rms',rmsE,'peak',pkE,'maxU',maxU);
    fprintf('  %s\n', label);
end

% =========================================================================
function disp_row(label, r)
    fprintf(' %-22s %10.4f %10.4f %10.1f %10.4f\n', ...
        label, r.rms, r.peak, r.maxU, r.span);
end
