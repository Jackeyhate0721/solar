function sweep_sensitivity()
%SWEEP_SENSITIVITY Where does tracking win, and where does it lose?
%
%   Two analyses:
%   A) Clear-sky daily gain of single- and dual-axis trackers over a
%      fixed-tilt panel, as a function of latitude and day of year.
%   B) Expected annual gain vs. the fraction of overcast (diffuse-only)
%      days, at a chosen latitude -- the "is tracking worth it in a cloudy
%      climate" trade-off:
%          gain(f) = (1-f) * gain_clear + f * gain_overcast
%      with each sky state simulated for one representative day.

close all; clc;

% ---- plant / drive parameters (same as run_tracker) ----
panel.A = 2.0; panel.m = 22; panel.r = 0.5; g = 9.81;
act.Tf0   = 1.5; act.b = 0.10; act.eta = 0.60;
act.wmax  = deg2rad(3);
wind.k    = 0.5*1.225*1.2*panel.A*0.75;
albedo    = 0.2;

% plant is shared across all configs and scenarios
plant = struct('act',act,'panel',panel,'wind',wind, ...
               'albedo',albedo,'g',g);

dt   = 5/60;                 % h
t    = 0:dt:24-dt;           % h
tsec = dt*3600;

% deterministic sky / wind states
vcalm    = 2.5 + 0*t;
ktClear  = 0.85 + 0*t;
ktOvcst  = 0.15 + 0*t;

% =========================================================================
% A) Latitude x day-of-year heatmap of tracking gain (clear sky)
% =========================================================================
latGrid = 0:5:60;
doyGrid = 1:15:365;

gainS = nan(numel(latGrid), numel(doyGrid));
gainD = nan(numel(latGrid), numel(doyGrid));

for i = 1:numel(latGrid)
    lat = latGrid(i);
    for j = 1:numel(doyGrid)
        doy = doyGrid(j);
        [alpha, gamma_s, theta_z] = sun_position(lat, doy, t);
        [Gbn, Gd_h, Gh] = gen_irradiance(doy, theta_z, ktClear);
        env = struct('alpha',alpha,'gamma_s',gamma_s,'theta_z',theta_z, ...
                     'Gbn',Gbn,'Gd_h',Gd_h,'Gh',Gh,'kt',ktClear,'v',vcalm, ...
                     't',t,'tsec',tsec);

        oF = simulate_tracker(makecfg('fixed',lat), env, plant);
        o1 = simulate_tracker(makecfg('hsa', lat), env, plant);
        o2 = simulate_tracker(makecfg('dual', lat), env, plant);

        gainS(i,j) = (o1.Enet - oF.Enet)/oF.Enet*100;
        gainD(i,j) = (o2.Enet - oF.Enet)/oF.Enet*100;
    end
end

figure('Name','Tracking gain map','Position',[80 120 1150 460]);
subplot(1,2,1);
imagesc(doyGrid, latGrid, gainS); axis xy; colorbar; colormap(turbo);
xlabel('Day of year'); ylabel('Latitude [deg N]');
title('Single-axis gain over fixed, clear sky [%]');
subplot(1,2,2);
imagesc(doyGrid, latGrid, gainD); axis xy; colorbar; colormap(turbo);
xlabel('Day of year'); ylabel('Latitude [deg N]');
title('Dual-axis gain over fixed, clear sky [%]');

% =========================================================================
% B) Breakeven vs fraction of overcast days, at a chosen latitude
% =========================================================================
lat  = 34;
days = [172 80];               % summer solstice, near equinox
names = {'Summer solstice' 'Near equinox'};

figure('Name','Cloudiness breakeven','Position',[120 160 900 420]);
lineStyles = {'-','--','-.'};
cols = lines(4);
gains = zeros(4, numel(days), 2);   % config x day x sky (clear, overcast)

for kd = 1:numel(days)
    doy = days(kd);
    [alpha, gamma_s, theta_z] = sun_position(lat, doy, t);
    [GbnC, GdC, GhC] = gen_irradiance(doy, theta_z, ktClear);
    [GbnO, GdO, GhO] = gen_irradiance(doy, theta_z, ktOvcst);

    envC = struct('alpha',alpha,'gamma_s',gamma_s,'theta_z',theta_z, ...
                  'Gbn',GbnC,'Gd_h',GdC,'Gh',GhC,'kt',ktClear,'v',vcalm, ...
                  't',t,'tsec',tsec);
    envO = struct('alpha',alpha,'gamma_s',gamma_s,'theta_z',theta_z, ...
                  'Gbn',GbnO,'Gd_h',GdO,'Gh',GhO,'kt',ktOvcst,'v',vcalm, ...
                  't',t,'tsec',tsec);
    modes = {'fixed','hsa','dual','smart'};
    E = zeros(4,2);                     % config x sky, absolute Wh/m2
    for c = 1:4
        cfg = makecfg(modes{c}, lat);
        oC = simulate_tracker(cfg, envC, plant);
        oO = simulate_tracker(cfg, envO, plant);
        E(c,1) = oC.Enet; E(c,2) = oO.Enet;
    end
    gains(:,kd,1) = (E(:,1) - E(1,1))/E(1,1)*100;   % clear-sky gain
    gains(:,kd,2) = (E(:,2) - E(1,2))/E(1,2)*100;   % overcast gain

    % expected gain vs fraction f of overcast days
    f = 0:0.01:1;
    subplot(1,2,kd); hold on; grid on;
    for c = 2:4
        gC = gains(c,kd,1); gO = gains(c,kd,2);
        plot(f*100, (1-f)*gC + f*gO, lineStyles{c-1}, 'LineWidth', 1.6, ...
             'Color', cols(c,:));
        fprintf('%s, lat %d: %s  clear %+.1f%%  overcast %+.1f%%\n', ...
            names{kd}, lat, makecfg(modes{c},lat).label, gC, gO);
    end
    yline(0, 'k:', 'LineWidth', 1);
    xlabel('Overcast days fraction [%]'); ylabel('Expected gain over fixed [%]');
    title(names{kd}, 'Interpreter','none');
end
legend({'Single-axis','Dual-axis','Smart dual-axis'},'Location','best');

exportgraphics(gcf,'fig_sensitivity.png','Resolution',110);

end

% -------------------------------------------------------------------------
function cfg = makecfg(mode, lat)
    % Deadband is per-configuration: only smart uses a non-zero value.
    switch mode
        case 'fixed'
            cfg = struct('mode','fixed','label','Fixed', ...
                         'beta',deg2rad(0.9*lat),'gp',deg2rad(180), ...
                         'ktmin',0.30,'vstow',12,'deadband',0);
        case 'hsa'
            cfg = struct('mode','hsa','label','Single-axis', ...
                         'beta',0,'gp',0,'ktmin',0.30,'vstow',12,'deadband',0);
        case 'dual'
            cfg = struct('mode','dual','label','Dual-axis', ...
                         'beta',0,'gp',0,'ktmin',0.30,'vstow',12,'deadband',0);
        case 'smart'
            cfg = struct('mode','smart','label','Smart dual-axis', ...
                         'beta',0,'gp',0,'ktmin',0.30,'vstow',12, ...
                         'deadband',deg2rad(2.0));
    end
end