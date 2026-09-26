function kt = gen_kt(kind, t, seed)
%GEN_KT Smooth clearness-index time series for a day.
    rng(seed);
    switch kind
        case 'clear'
            base = 0.82; amp = [0.10 0.05 0.03]; frq = [2*pi/8 2*pi/3.2 2*pi/1.3];
        case 'partly'
            base = 0.55; amp = [0.25 0.20 0.12]; frq = [2*pi/7 2*pi/3 2*pi/1.1];
        otherwise
            base = 0.22; amp = [0.08 0.05 0.03]; frq = [2*pi/9 2*pi/3.5 2*pi/1.6];
    end
    ph = 2*pi*rand(1,3);
    kt = base;
    for i = 1:3
        kt = kt + amp(i)*sin(frq(i)*t + ph(i));
    end
    kt = kt + 0.05*randn(size(t));
    kt = smoothdata(kt,'movmean',4);
    kt = min(0.88, max(0.05, kt));
end
