classdef test_tracker < matlab.unittest.TestCase
%TEST_TRACKER A small matlab.unittest suite for the tracker project.
%
%   Two of the assertions here would have caught the two worst bugs reported
%   in the review: that 'hsa' capture differs from a flat fixed panel (the
%   old 'hsa' pinned the tilt at zero, making the azimuth rotation a no-op),
%   and that peak motor current is nonzero (the old tracking_day.m:128
%   reported max(abs(diff([i 0]))), always 0).

    methods (Test)
        function testHSAbeatsFlatFixed(obj)
            % The single-axis tracker must capture more energy than a fixed-
            % tilt panel (south-facing at 0.9*lat). On a clear day the HSA
            % sweeps E->W and beats the fixed tilt on beam capture; the old
            % code pinned the tilt at zero and returned the same value as a
            % flat fixed panel, which is what this assertion guards against.
            lat = 34; doy = 172; albedo = 0.2;
            panel.A = 2.0; panel.m = 22; panel.r = 0.5; g = 9.81;
            act.Tf0 = 1.5; act.b = 0.10; act.eta = 0.60;
            act.wmax = deg2rad(3);
            wind.k = 0.5*1.225*1.2*panel.A*0.75;
            dt = 5/60; t = 0:dt:24-dt; tsec = dt*3600;
            [alpha, gamma_s, theta_z] = sun_position(lat, doy, t);
            kt = 0.85 + 0*t; v = 2.5 + 0*t;
            [Gbn, Gd_h, Gh] = gen_irradiance(doy, theta_z, kt);

            env = struct('alpha',alpha,'gamma_s',gamma_s,'theta_z',theta_z, ...
                         'Gbn',Gbn,'Gd_h',Gd_h,'Gh',Gh,'kt',kt,'v',v, ...
                         't',t,'tsec',tsec);
            plant = struct('act',act,'panel',panel,'wind',wind, ...
                           'albedo',albedo,'g',g);

            oHSA = simulate_tracker(struct('mode','hsa','beta',0,'gp',0,...
                'ktmin',0.30,'vstow',12,'deadband',0), env, plant);
            oFixed = simulate_tracker(struct('mode','fixed',...
                'beta',deg2rad(0.9*lat),'gp',deg2rad(180), ...
                'ktmin',0.30,'vstow',12,'deadband',0), env, plant);

            obj.verifyGreaterThan(oHSA.Ecap, oFixed.Ecap, ...
                'hsa should capture more than a fixed-tilt panel on a clear day');
        end

        function testPeakCurrentNonzero(obj)
            metrics = tracking_day;
            obj.verifyGreaterThan(metrics.peakCurrentA, 0, ...
                'peak motor current must be nonzero');
        end

        function testSunPositionSanity(obj)
            % Summer solstice at solar noon: sun is high (alpha > 0, < 90)
            % and roughly due south. The Cooper + NOAA-azimuth convention
            % used here gives gamma_s within a couple of degrees of pi.
            [alpha, gamma_s, ~] = sun_position(34, 172, 12);
            obj.verifyGreaterThan(alpha, 0, ...
                'summer noon: alpha must be positive');
            obj.verifyLessThan(alpha, pi/2, ...
                'summer noon: alpha must be < 90');
            obj.verifyEqual(gamma_s, pi, 'AbsTol', 0.05, ...
                'summer noon: azimuth should be ~south (within 3 deg)');

            % Winter solstice at 6 am: at lat 34 the sun is still below the
            % horizon (Cooper declination + equation of time gives sunrise
            % near 7 am), so alpha is negative. The sign is the point:
            % the model must give negative elevation before sunrise.
            [alpha2, ~, ~] = sun_position(34, 355, 6);
            obj.verifyLessThan(alpha2, 0, ...
                'winter 6am: sun should be below the horizon at lat 34');
        end

        function testPanelGainIncidenceAngle(obj)
            Gbn = 800; Gd = 100; Gh = 900;
            theta_z = 0; gamma_s = pi; albedo = 0.2;
            beta = pi/2; gamma_p = pi;
            [~, cosInc] = panel_gain(Gbn, Gd, Gh, theta_z, gamma_s, beta, gamma_p, albedo);
            obj.verifyEqual(cosInc, 0, 'AbsTol', 1e-6, ...
                'vertical panel facing horizon: cosInc should be 0');

            theta_z2 = pi/2; beta2 = 0; gamma_p2 = pi;
            [~, cosInc2] = panel_gain(Gbn, Gd, Gh, theta_z2, gamma_s, beta2, gamma_p2, albedo);
            obj.verifyEqual(cosInc2, 0, 'AbsTol', 1e-6, ...
                'horizontal panel, sun at horizon: cosInc should be 0');
        end
    end
end
