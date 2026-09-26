function [Gp, cosInc] = panel_gain(Gbeam, Gd, Ghoriz, theta_z, gamma_s, beta, gamma_p, albedo)
%PANEL_GAIN Irradiance on a tilted panel [W/m^2].
%   [Gp, cosInc] = panel_gain(Gbeam, Gd, Ghoriz, theta_z, gamma_s, beta, gamma_p, albedo)
%
%   - Gbeam  : direct normal irradiance [W/m2]
%   - Gd     : diffuse horizontal irradiance [W/m2]
%   - Ghoriz : global horizontal irradiance [W/m2] (for ground reflection)
%   - theta_z: solar zenith angle [rad]
%   - gamma_s: solar azimuth, clockwise from north [rad]
%   - beta   : panel tilt from horizontal [rad]
%   - gamma_p: panel azimuth, clockwise from north [rad]
%   - albedo : ground reflectance
%
%   Isotropic sky-diffuse + ground-reflected model (Liu & Jordan).

    alpha  = pi/2 - theta_z;

    % Angle of incidence between sun and panel normal
    cosInc = sin(alpha).*cos(beta) + cos(alpha).*sin(beta).*cos(gamma_s - gamma_p);
    cosInc = max(cosInc, 0);

    G_beam = Gbeam .* cosInc;                       % beam on panel
    G_sky  = Gd     .* (1 + cos(beta))/2;           % isotropic diffuse
    G_gnd  = albedo .* Ghoriz .* (1 - cos(beta))/2; % ground reflection

    Gp = G_beam + G_sky + G_gnd;
end
