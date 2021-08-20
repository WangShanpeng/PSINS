function xy = xyrot(xy, yawrot)
% XY coordinate rotation by Z axis.
%
% Prototype: xy = xyrot(xy, yawrot)
% Inputs: xy - xy coordinate
%         yawrot - yaw rotation angle in rad, positive for counter-clockwise
% Output: xy - xy coordinate after rotation
%
% See also: apmove, avprot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/01/2021
    rot = [  cos(yawrot),  sin(yawrot); 
            -sin(yawrot),  cos(yawrot) ];
    xy(:,1:2) = xy(:,1:2) * rot;
    
    