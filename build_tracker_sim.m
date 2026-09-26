function build_tracker_sim()
%BUILD_TRACKER_SIM Rebuild the Simulink/Simscape tracker model cleanly.
%   The controller has real authority over the plant: a PID on the panel ANGLE
%   (radians) drives a Controlled Voltage Source that powers an ideal PM DC
%   motor (armature R + L + Rotational Electromechanical Converter, K = Kt)
%   through the worm gear. Wind and gravity enter as genuine torque
%   disturbances on the panel node (not the old hard-coded zero).
%
%   Every block is referenced by its explicit library path and every
%   set_param is a plain call, so a renamed block or parameter fails loudly
%   instead of silently producing a wrong model.

mdl = 'tracker_sim';
lib = fullfile(pwd, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(lib, 'file') == 4, delete(lib); end
new_system(mdl);

p = servo_params();
add_blocks(mdl);
set_params(mdl, p);
wire(mdl);
save_system(mdl, lib);
close_system(mdl, 0);
fprintf('Built %s\n', lib);
end

% -------------------------------------------------------------------------
function add_blocks(mdl)
    % Full explicit library paths (R2023b) -- no brute-force library search.
    load_system('fl_lib'); load_system('ee_lib'); load_system('nesl_utility');
    load_system('sdl_lib'); load_system('simulink');

    % --- Simscape Driveline transmission ---
    add_block('sdl_lib/Gears/Worm Gear',                         [mdl '/Worm Gear']);
    % --- Simscape Foundation mechanical panel plant ---
    add_block('fl_lib/Mechanical/Rotational Elements/Inertia',                [mdl '/PanelJ']);
    add_block('fl_lib/Mechanical/Rotational Elements/Rotational Damper',      [mdl '/PanelDamper']);
    add_block('fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference', [mdl '/RotRef']);
    add_block('fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor',   [mdl '/MotionSensor']);
    % --- torque disturbance sources (wind + gravity) on the panel node ---
    add_block('fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source',     [mdl '/WindTS']);
    add_block('fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source',      [mdl '/GravTS']);
    % --- PM DC motor = Controlled Voltage Source + armature R,L + ideal converter + rotor inertia ---
    add_block('fl_lib/Electrical/Electrical Sources/Controlled Voltage Source', [mdl '/CVsrc']);
    add_block('fl_lib/Electrical/Electrical Elements/Resistor',                 [mdl '/ArmR']);
    add_block('fl_lib/Electrical/Electrical Elements/Inductor',                 [mdl '/ArmL']);
    add_block('fl_lib/Electrical/Electrical Elements/Rotational Electromechanical Converter', [mdl '/Motor']);
    add_block('fl_lib/Mechanical/Rotational Elements/Inertia',                  [mdl '/MotorJ']);
    add_block('ee_lib/Connectors & References/Electrical Reference',           [mdl '/Eref']);
    % --- Simscape utility ---
    add_block('nesl_utility/Solver Configuration',  [mdl '/SolverConf']);
    add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS2Sim']);   % angle  PS -> Simulink
    add_block('nesl_utility/Simulink-PS Converter', [mdl '/Sim2PS_V']); % voltage Simulink -> PS
    add_block('nesl_utility/Simulink-PS Converter', [mdl '/Sim2PS_W']); % wind    Simulink -> PS
    add_block('nesl_utility/Simulink-PS Converter', [mdl '/Sim2PS_G']); % gravity Simulink -> PS
    % --- Simulink signal path: PID on panel angle (rad) ---
    add_block('simulink/Sources/From Workspace',       [mdl '/Ref']);       % ref_tilt  [t, rad]
    add_block('simulink/Sources/From Workspace',       [mdl '/Wind']);      % wind_tq [t, N*m]
    add_block('simulink/Math Operations/Sum',          [mdl '/SumErr']);    % ref - angle
    add_block('simulink/Continuous/PID Controller',    [mdl '/PID']);
    add_block('simulink/Discontinuities/Saturation',  [mdl '/Sat']);       % clamp to +/- Vmax [V]
    add_block('simulink/Discrete/Memory',             [mdl '/Mem']);       % break gravity algebraic loop
    add_block('simulink/Math Operations/Trigonometric Function', [mdl '/Cos']);
    add_block('simulink/Math Operations/Gain',        [mdl '/GainGrav']);  % m*g*r*cos(theta)
    add_block('simulink/Sinks/To Workspace',          [mdl '/OutAngle']);
    add_block('simulink/Sinks/To Workspace',          [mdl '/OutErr']);
    add_block('simulink/Sinks/To Workspace',          [mdl '/OutVolt']);
end

% -------------------------------------------------------------------------
function wire(mdl)
    ph = @(nm) get_param([mdl '/' nm], 'PortHandles');
    % --- armature electrical loop ---
    % Controlled Voltage Source port map (verified): LConn(1)=electrical +,
    % RConn(1)=PS voltage control, RConn(2)=electrical -. The PID output
    % (volts) -> Sim2PS_V -> RConn(1). Loop: CVsrc+ -> R -> L -> Motor(+);
    % Motor(-) -> CVsrc- ; Electrical Reference on the - node.
    add_conn(mdl, ph('CVsrc').LConn(1),   ph('ArmR').LConn(1));
    add_conn(mdl, ph('ArmR').RConn(1),    ph('ArmL').LConn(1));
    add_conn(mdl, ph('ArmL').RConn(1),    ph('Motor').LConn(1));
    add_conn(mdl, ph('Motor').RConn(1),   ph('CVsrc').RConn(2));
    add_conn(mdl, ph('Eref').LConn(1),    ph('CVsrc').RConn(2));
    add_conn(mdl, ph('Sim2PS_V').RConn(1), ph('CVsrc').RConn(1));
    % --- mechanical network ---
    % Motor converter: LConn(2)=shaft -> worm gear; RConn(2)=case -> ground.
    % Rotor inertia MotorJ sits on the shaft node.
    add_conn(mdl, ph('Motor').LConn(2),   ph('Worm Gear').LConn(1));
    add_conn(mdl, ph('Motor').RConn(2),   ph('RotRef').LConn(1));
    add_conn(mdl, ph('MotorJ').LConn(1),  ph('Worm Gear').LConn(1));
    % Worm gear output -> panel node (inertia, damper, sensor, disturbances).
    add_conn(mdl, ph('Worm Gear').RConn(1), ph('PanelJ').LConn(1));
    add_conn(mdl, ph('PanelDamper').LConn(1), ph('Worm Gear').RConn(1));
    add_conn(mdl, ph('PanelDamper').RConn(1), ph('RotRef').LConn(1));
    add_conn(mdl, ph('MotionSensor').RConn(1), ph('Worm Gear').RConn(1));
    add_conn(mdl, ph('MotionSensor').LConn(1), ph('RotRef').LConn(1));
    % Motion sensor PS outputs default to [W, phi]; RConn(3) = panel ANGLE.
    add_conn(mdl, ph('MotionSensor').RConn(3), ph('PS2Sim').LConn(1));
    % Wind + gravity torque sources: one mech port on the panel node, the
    % other on ground -- so they actually apply torque (the old model wired
    % both ports of a torque source to the SAME node, contributing zero).
    add_conn(mdl, ph('WindTS').LConn(1),  ph('Worm Gear').RConn(1));
    add_conn(mdl, ph('WindTS').RConn(2),  ph('RotRef').LConn(1));
    add_conn(mdl, ph('Sim2PS_W').RConn(1), ph('WindTS').RConn(1));
    add_conn(mdl, ph('GravTS').LConn(1),  ph('Worm Gear').RConn(1));
    add_conn(mdl, ph('GravTS').RConn(2),  ph('RotRef').LConn(1));
    add_conn(mdl, ph('Sim2PS_G').RConn(1), ph('GravTS').RConn(1));
    % Solver ground
    add_conn(mdl, ph('SolverConf').RConn(1), ph('RotRef').LConn(1));
    % --- Simulink signals ---
    add_sig(mdl, 'Ref/1',      'SumErr/1');   % reference  [rad]
    add_sig(mdl, 'PS2Sim/1',   'SumErr/2');   % feedback angle [rad]
    add_sig(mdl, 'SumErr/1',   'PID/1');      % error -> PID
    add_sig(mdl, 'PID/1',      'Sat/1');      % command voltage
    add_sig(mdl, 'Sat/1',      'Sim2PS_V/1'); % -> controlled source
    add_sig(mdl, 'Sat/1',      'OutVolt/1');  % log voltage
    add_sig(mdl, 'PS2Sim/1',   'OutAngle/1'); % log panel angle
    add_sig(mdl, 'SumErr/1',   'OutErr/1');   % log tracking error
    % gravity torque = m*g*r*cos(theta); Memory breaks the algebraic loop
    % (gravity depends on angle, angle depends on gravity).
    add_sig(mdl, 'PS2Sim/1',   'Mem/1');
    add_sig(mdl, 'Mem/1',      'Cos/1');
    add_sig(mdl, 'Cos/1',      'GainGrav/1');
    add_sig(mdl, 'GainGrav/1', 'Sim2PS_G/1');
end

% -------------------------------------------------------------------------
function set_params(mdl, p)
    % --- panel plant ---
    set_param([mdl '/PanelJ'],      'inertia', num2str(p.Jl));
    set_param([mdl '/PanelDamper'], 'D', num2str(p.blms));
    % --- worm gear (60:1) ---
    set_param([mdl '/Worm Gear'],   'ratio', num2str(p.N));
    % --- PM DC motor: armature R,L + ideal converter (K = Kt = Ke) + rotor J ---
    set_param([mdl '/ArmR'],  'R', num2str(p.R));
    set_param([mdl '/ArmL'],  'L', num2str(p.L));
    set_param([mdl '/Motor'], 'K', num2str(p.Kt));
    set_param([mdl '/MotorJ'], 'inertia', num2str(p.Jm));

    % --- PID (on angle in rad; output is motor voltage in V, clamped to Vmax) ---
    set_param([mdl '/PID'], 'P','90','I','4','D','5','N','20', ...
                              'LimitOutput','on', ...
                              'UpperSaturationLimit', num2str(p.Vmax), ...
                              'LowerSaturationLimit', num2str(-p.Vmax), ...
                              'AntiWindupMode','back-calculation');
    set_param([mdl '/Sat'], 'UpperLimit', num2str(p.Vmax), ...
                            'LowerLimit', num2str(-p.Vmax));
    % gravity gain (sign: restoring torque that the motor must overcome)
    set_param([mdl '/GainGrav'], 'Gain', num2str(-p.m*p.g*p.r));
    set_param([mdl '/Cos'],      'Operator', 'cos');

    % --- workspace I/O ---
    set_param([mdl '/Ref'],  'VariableName','ref_tilt', 'SampleTime','0.02', ...
                             'Interpolate','on');
    set_param([mdl '/Wind'], 'VariableName','wind_tq',  'SampleTime','0.02', ...
                             'Interpolate','on');
    set_param([mdl '/OutAngle'], 'VariableName','panel_ang','SaveFormat','Array');
    set_param([mdl '/OutErr'],   'VariableName','track_err','SaveFormat','Array');
    set_param([mdl '/OutVolt'],  'VariableName','motor_v', 'SaveFormat','Array');

    % --- solver: ode23t for the stiff Simscape network, bounded step ---
    set_param(mdl, 'StopTime','6*3600', 'Solver','ode23t', 'MaxStep','0.1');
end

% --- helpers ---
function add_conn(mdl, p1, p2)
%ADD_CONN  physical (Simscape) connection.
    add_line(mdl, p1, p2);
end

function add_sig(mdl, s, d)
%ADD_SIG  Simulink signal connection by block/port.
    add_line(mdl, s, d, 'autorouting', 'on');
end
