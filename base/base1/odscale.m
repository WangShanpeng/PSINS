function kod = odscale(od, distance)
% Odometer scale factor calibration by distance or GNSS.
%
% Prototype: kod = odscale(od, distance)
% Inputs: od - od data, od(:,1) is looked as distance increament
%         distance - distance from od(1) to od(end)
% Output: kod - odometer scale factor.
%
% Example:
%   gpsavp = gps2avp(gps); insplot(gpsavp);
%   kod = odscale(datacut(odo,115,130),gps)
%
% See also  drcalibrate, drinit, drupdate, drrectify.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/01/2022
    if length(distance)>3  % kod = odscale(od, gps)
        gps = distance;
        p1 = getat(gps, od(1,end));  p2 = getat(gps, od(end,end));
        dxyz = pos2dxyz([p1(end-2:end)'; p2(end-2:end)']);
        distance = norm(dxyz(end,1:2));
    end
    kod = distance/sum(od(:,1));
