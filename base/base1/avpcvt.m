function avp = avpcvt(avp, deg2rad)
% avp convension from degree to rad.
%
% Prototype: avp = avpcvt(avp, deg2rad)
% Inputs: avp - att/lat/lon in degree or rad,
%               where yaw ranging [0,360]*deg clockwise or [-pi,pi] conter-clockwise
%         deg2rad - =1 for deg2rad; =0 for rad2deg
% Output: avp - if deg2rad==1; att/lat/lon in rad, 
%               where yaw ranging [-pi,pi] conter-clockwise;
%               if deg2rad==0; att/lat/lon in deg, 
%               where yaw ranging [0,360]*deg clockwise
%
% See also  avpidx, yawcvt.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/03/2018, 29/12/2020
global glv
    if nargin<2, deg2rad=1; end
    if deg2rad==1,
        avp(:,[1:3,7:8]) = avp(:,[1:3,7:8])*glv.deg;
        avp(:,3) = yawcvt(avp(:,3),'c360cc180');
    else
        avp(:,3) = yawcvt(avp(:,3),'cc180c360');
        avp(:,[1:3,7:8]) = avp(:,[1:3,7:8])/glv.deg;
    end