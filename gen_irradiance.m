function [Gbn, Gd_h, Gh] = gen_irradiance(doy, theta_z, kt)
%GEN_IRRADIANCE Direct-normal and diffuse-horizontal irradiance.
%   Global horizontal irradiance scaled by a clearness index kt, with the
%   beam/diffuse split from the Erbs correlation.
    Gsc = 1367;
    G0  = Gsc*(1 + 0.033*cosd(360*doy/365));

    night = theta_z > deg2rad(87);
    nz    = ~night;
    cz    = cos(theta_z);
    cz(night) = 0;
    cz(cz < 0.02 & nz) = 0.02;

    Gh = G0.*cz.*kt;
    Gh(night) = 0;

    % Erbs diffuse fraction
    ktc = min(max(kt,0.02),0.9);
    fd  = zeros(size(ktc));
    lo  = ktc < 0.22;
    hi  = ktc > 0.8;
    mid = ~lo & ~hi;
    fd(lo)  = 1 - 0.09*ktc(lo);
    fd(mid) = 0.9511 - 0.1604*ktc(mid) + 4.388*ktc(mid).^2 ...
              - 16.638*ktc(mid).^3 + 12.336*ktc(mid).^4;
    fd(hi)  = 0.165;
    fd      = min(fd, 0.95);

    Gd_h = fd.*Gh;
    Gb_h = max(Gh - Gd_h, 0);

    Gbn = zeros(size(Gb_h));
    Gbn(nz) = Gb_h(nz)./cz(nz);
end
