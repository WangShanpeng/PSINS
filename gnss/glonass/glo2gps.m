function XYZgps = glo2gps(XYZglo)
% Coordinate transformation from GLONASS ECEF frame to GPS ECEF frame.
%
% Prototype: XYZgps = glo2gps(XYZglo) 
% Input: XYZglo - coordinates in GLONASS ECEF frame
% Output: XYZgps - coordinates in GPS ECEF frame
%
% See also  xyz2blh.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/08/2015
    M = [1,-1.728e-6,-0.005e-6; 1.728e-6,1,-0.012e-6; 0.005e-6,0.012e-6,1];
    S = 1+22e-9;
    dXYZ = [-0.47;0.51;-2.0];
    if size(XYZglo,2)==1
        XYZgps = dXYZ+S*(M*XYZglo);
    else
        XYZgps = S*(XGZglo*M');
        XYZgps = [XYZgps(:,1)+dXYZ(1), XYZgps(:,2)+dXYZ(2), XYZgps(:,3)+dXYZ(3)];
    end