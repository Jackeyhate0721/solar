function [alpha, gamma_s, theta_z] = sun_position(lat_deg, doy, t_hour)
%SUN_POSITION Solar elevation, azimuth and zenith angle.
%   [alpha, gamma_s, theta_z] = sun_position(lat_deg, doy, t_hour)
%
%   - lat_deg : latitude in degrees (positive = north)
%   - doy     : day of year (1..365)
%   - t_hour  : clock hour(s) of day (0..24), can be an array
%
%   Outputs (radians):
%   - alpha   : solar elevation angle
%   - gamma_s : solar azimuth, measured clockwise from north
%               (0=N, 90=E, 180=S, 270=W)
%   - theta_z : solar zenith angle
%
%   Uses the Cooper declination approximation and the equation of time;
%   azimuth quadrant follows the NOAA convention.

    lat   = deg2rad(lat_deg);
    decl  = deg2rad(23.45) * sind(360*(284 + doy)/365);

    % Equation of time (minutes)
    B   = deg2rad(360*(doy - 81)/364);
    EoT = 9.87*sin(2*B) - 7.53*cos(B) - 1.5*sin(B);

    % True solar time (observer assumed on its standard meridian)
    t_solar = t_hour + EoT/60;

    % Hour angle (rad)
    H = deg2rad(15*(t_solar - 12));

    % Elevation
    sinAlpha = sin(lat)*sin(decl) + cos(lat)*cos(decl)*cos(H);
    sinAlpha = min(1, max(-1, sinAlpha));
    alpha    = asin(sinAlpha);
    theta_z  = pi/2 - alpha;

    % Azimuth from north, clockwise (NOAA)
    cosAz = (sin(decl) - sin(alpha).*sin(lat)) ./ (cos(alpha).*cos(lat));
    cosAz = min(1, max(-1, cosAz));
    gamma_s       = acos(cosAz);            % morning / noon quadrant
    gamma_s(H>0)  = 2*pi - gamma_s(H>0);    % afternoon quadrant
end
