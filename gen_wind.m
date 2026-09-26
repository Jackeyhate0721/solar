function v = gen_wind(kind, t, seed)
%GEN_WIND Wind-speed time series (m/s).
    rng(seed + 11);
    if strcmp(kind,'calm')
        v = 2.2 + 0.8*sin(2*pi*t/24 + 1) + 0.5*randn(size(t));
    else
        v = 5.5 + 2.2*sin(2*pi*t/12 + 0.7) + 2.0*sin(2*pi*t/2.9 + 0.3) ...
            + 1.5*exp(-mod(t + 0.2, 0.35)*12) + 0.8*randn(size(t));
    end
    v = max(0.5, smoothdata(v,'movmean',3));
end
