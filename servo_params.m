function p = servo_params()
%SERVO_PARAMS Lumped elevation-axis servo parameters.
%   Two-mass inertia model: DC motor -> 60:1 gearbox (with backlash) ->
%   torsionally compliant shaft -> panel inertia, plus gravity and wind
%   load at the panel. All quantities SI, referenced to the panel axis
%   where noted.

    % ---- motor (simplified permanent-magnet DC) ----
    p.Kt   = 0.100;     % motor torque constant          [N*m/A]
    p.Ke   = 0.100;     % back-EMF constant              [V*s/rad]
    p.R    = 1.0;       % armature resistance            [ohm]
    p.L    = 1e-3;      % armature inductance            [H]
    p.Jm   = 2.0e-4;    % rotor inertia                  [kg*m^2]
    p.bm   = 2e-4;      % rotor viscous damping          [N*m*s/rad]
    p.Tcm  = 0.02;      % motor Coulomb friction         [N*m]

    % ---- gearbox ----
    p.N    = 60;        % speed ratio (motor : panel)
    p.eta  = 0.90;      % gearbox efficiency

    % ---- shaft compliance and backlash between gearbox and panel ----
    p.ks   = 5000;      % torsional stiffness            [N*m/rad]
    p.cs   = 50;        % torsional damping               [N*m*s/rad]
    p.bl   = deg2rad(0.2);  % backlash half-gap at panel axis [rad]

    % ---- panel (load) ----
    p.m    = 22;        % module mass                    [kg]
    p.r    = 0.5;       % CG offset from tilt axis       [m]
    p.g    = 9.81;
    p.Jl   = 2.0;       % panel inertia about tilt axis  [kg*m^2]
    p.blms = 0.5;       % panel bearing viscous damping  [N*m*s/rad]
    p.Tcp  = 0.5;       % panel Coulomb friction         [N*m]

    % ---- drive limits (48 V gives ~4.8 A*m stall -> ~260 N*m at panel) ----
    p.Vmax = 48;        % driver supply voltage          [V]

    % motor-side torque from armature voltage (assume current loop lamped)
end