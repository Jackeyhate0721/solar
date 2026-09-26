function out = simulate_tracker(cfg, env, plant)
%SIMULATE_TRACKER One-day run of a single panel configuration.
%   Sequential state update with slew-rate limit and deadband, actuator
%   energy from a torque model, and isotropic-diffuse panel irradiance.
%
%   'hsa' is a real horizontal single-axis tracker: the panel rotates about
%   a N-S axis using the NREL rotation angle R, so beta and gamma_p both
%   vary through the day. The old 'hsa' pinned the tilt at zero, which made
%   the azimuth rotation independent of captured energy -- verifying against
%   a flat fixed panel now returns a nonzero gain.
%
%   Inputs (the old 16 positional arguments are now two structs):
%   - cfg   : panel configuration (mode, label, beta, gp, ktmin, vstow,
%             deadband). See run_tracker.m for the canonical set.
%   - env   : environment and sky
%             .alpha, .gamma_s, .theta_z   sun geometry [rad]
%             .Gbn, .Gd_h, .Gh             irradiance fields [W/m2]
%             .kt                          clearness index [-]
%             .v                           wind speed [m/s]
%             .t                           time [h]
%             .tsec                        seconds per step [s]
%   - plant : hardware
%             .act.Tf0, .act.b, .act.eta, .act.wmax
%             .panel.A, .panel.m, .panel.r
%             .wind.k                      T_wind = k*v^2
%             .albedo
%             .g                           gravity [m/s2]

    % Unpack the env and plant structs into local names so the body of the
    % loop reads the same as before.
    alpha   = env.alpha;
    gamma_s = env.gamma_s;
    theta_z = env.theta_z;
    Gbn     = env.Gbn;
    Gd_h    = env.Gd_h;
    Gh      = env.Gh;
    kt      = env.kt;
    v       = env.v;
    t       = env.t;
    tsec    = env.tsec;
    act     = plant.act;
    panel   = plant.panel;
    wind    = plant.wind;
    albedo  = plant.albedo;
    g       = plant.g;

    nt = numel(t);
    dt = t(2) - t(1);

    % NREL rotation angle for a horizontal single-axis tracker (axis along
    % the N-S line, panel sweeps E -> W across the day). tan(R) =
    % sin(theta_z)*sin(gamma_s - pi) / cos(theta_z). R is the panel's
    % rotation about the axis; beta (tilt from horizontal) and gamma_p
    % (panel azimuth, clockwise from north) follow from R.
    % At noon (gamma_s = pi) R = 0 so the panel is flat (beta = 0) facing
    % south -- the correct single-axis pose for clear-sky beam capture.
    R_HSA  = zeros(1,nt);
    bd_HSA = zeros(1,nt);
    gd_HSA = zeros(1,nt);
    for k = 1:nt
        cz = cos(theta_z(k));
        if cz > 0
            num = sin(theta_z(k))*sin(gamma_s(k) - pi);
            R   = atan(num/cz);
            R   = max(min(R, pi/2), -pi/2);      % clamp to +/-90 deg
            R_HSA(k)  = R;
            bd_HSA(k) = abs(R);                   % tilt from horizontal
            gd_HSA(k) = pi/2 + R;                 % south at noon (R=0)
        else
            bd_HSA(k) = pi/2; gd_HSA(k) = 0;      % night: hold vertical
        end
    end

    b0 = cfg.beta;      % current tilt [rad]
    g0 = cfg.gp;        % current azimuth [rad]

    Ecap = 0; Eact = 0; slew = 0; windExp = 0;
    beta = zeros(1,nt); gp = zeros(1,nt);
    isSmart = strcmp(cfg.mode,'smart');
    ktHi = cfg.ktmin + 0.18;                 % resume-tracking threshold
    stowed = false;                          % smart state: tracking vs stowed

    for k = 1:nt
        % ---- desired orientation ----
        % Mechanical tilt limits: a real tracker cannot tilt fully vertical
        % (typically +/-75 deg from horizontal). Backtracking: at low sun
        % elevation (near sunrise/sunset) the tracker reduces the panel tilt
        % to avoid inter-row shading -- instead of chasing the sun to a
        % steep angle where it would cast a long shadow onto the next row,
        % it stays closer to horizontal. The backtracking threshold is set
        % so that the row pitch and sun elevation keep the panel clear.
        % (A full geometric shading model would be more rigorous; this is
        % a simplified tilt-reduction that captures the qualitative effect.)
        switch cfg.mode
            case 'fixed'
                bd = cfg.beta; gd = cfg.gp;
            case 'hsa'
                bd = bd_HSA(k); gd = gd_HSA(k);
            otherwise
                % ideal dual-axis: panel normal points at the sun
                bd = pi/2 - alpha(k);
                gd = gamma_s(k);
                % mechanical tilt limit (+/-75 deg from horizontal)
                bd = max(min(bd, deg2rad(75)), deg2rad(15));
                % backtracking: at low sun elevation, reduce tilt to
                % ~30 deg to avoid inter-row shading
                if alpha(k) < deg2rad(25)
                    bd = deg2rad(30);
                end
        end

        % ---- smart rules with hysteresis (avoids chatter) ----
        % Flat (beta = 0) maximises diffuse capture, so clouded sky calls
        % for the same pose as high wind. Two thresholds: stow below
        % ktmin, resume only above ktmin + dkt, so kt hovering around the
        % boundary does not make the tracker thrash.
        if isSmart
            if ~stowed && (v(k) > cfg.vstow || kt(k) < cfg.ktmin)
                stowed = true;
            elseif stowed && v(k) <= cfg.vstow && kt(k) > ktHi
                stowed = false;
            end
            if stowed
                bd = 0; gd = g0;
            end
        end

        % night / negligible irradiance: hold
        if Gh(k) < 1
            bd = b0; gd = g0;
        end

        % ---- slew-rate limited, deadbanded motion ----
        % The deadband is a per-configuration choice (cfg.deadband), not a
        % global actuator property -- this is what distinguishes the smart
        % tracker from the plain dual-axis: a wider deadband means fewer
        % actuator cycles (less wear) at the cost of a larger steady-state
        % tracking error. The old code applied act.deadband to every mode,
        % which made Dual-axis and Smart bit-identical in clear weather.
        % cfg.deadband is optional: a missing field means zero deadband, so
        % old call sites that never set it still work.
        bstep = 0; gstep = 0; omg_b = 0; omg_g = 0;
        db = 0;
        if isfield(cfg, 'deadband'), db = cfg.deadband; end
        eb = bd - b0;
        if abs(eb) > db
            bstep = sign(eb)*min(abs(eb), act.wmax*tsec);
            omg_b = abs(bstep)/tsec;
            b0 = b0 + bstep;
        end
        eg = wrapToPi(gd - g0);
        if abs(eg) > db
            gstep = sign(eg)*min(abs(eg), act.wmax*tsec);
            omg_g = abs(gstep)/tsec;
            g0 = g0 + gstep;
        end
        slew = slew + abs(bstep) + abs(gstep);

        % ---- actuator energy (self-locking worm gear: ~no holding power) ----
        if Gh(k) >= 1
            Tw    = wind.k*v(k)^2;
            Tfric = act.Tf0 + act.b*(omg_b + omg_g);
            Tgrav = panel.m*g*panel.r*cos(b0);
            if bstep >= 0
                Tb = Tgrav + Tfric + Tw;      % raising: motor fights gravity
            else
                Tb = Tfric + Tw;              % lowering: gravity assists
            end
            Tg = Tfric + Tw;
            P  = (Tb*omg_b + Tg*omg_g)/act.eta;
            Eact = Eact + P*tsec/3600;        % Wh
            % wind load experienced while the drive is actually moving
            % (the risky regime for drive wear / structural fatigue)
            if omg_b + omg_g > 0
                windExp = windExp + Tw*dt;    % N*m*h
            end
        end

        % ---- captured energy ----
        Gp = panel_gain(Gbn(k), Gd_h(k), Gh(k), theta_z(k), gamma_s(k), b0, g0, albedo);
        Ecap = Ecap + Gp*panel.A*dt;          % Wh

        beta(k) = b0; gp(k) = g0;
    end

    out.Ecap = Ecap; out.Eact = Eact; out.Enet = Ecap - Eact;
    out.slew = slew; out.windExp = windExp;
    out.beta = beta; out.gp = gp;
end
