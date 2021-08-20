function dkgz = rotz2dkgz(phi0, phi1, yaw0, yaw1, phi_mu)
% Estimate gyro install error dKG(:,3) when rotated by z axis from yaw0 to yaw1,
% where x&y axis is supposed to be levelling.
% Ref. My PhD. thesis Eq.(3.2-5).
%
% Prototype: dkgz = rotz2kgz(phi0, phi1, yaw0, yaw1, phi_mu)
% Inputs: phi0 - misalignment error at yaw0
%         phi1 - misalignment error at yaw1
%         yaw0, yaw1 - rotated by z axis from yaw0 to yaw1
%         phi_mu - =1 for phi, =0 for mu
% Output: dkgz - [dkgxz; dkgyz; dkgzz]
%
% See also  imuclbt, a2cwa.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/08/2020
    if nargin<5, phi_mu=1; end
    if phi_mu==0
%         phi0 = aa2phi([0;0;yaw0]+a2caw([0;0;yaw0])*phi0, [0;0;yaw0]);  % mu -> datt -> phi
%         phi1 = aa2phi([0;0;yaw1]+a2caw([0;0;yaw1])*phi1, [0;0;yaw1]);
        phi0 = mu2phi(phi0, [0;0;yaw0]);
        phi1 = mu2phi(phi1, [0;0;yaw1]);
    end
    sy0 = sin(yaw0); cy0 = cos(yaw0);
    sy1 = sin(yaw1); cy1 = cos(yaw1);
    dkgz = [  sy1-sy0    cy1-cy0   0
             -(cy1-cy0)  sy1-sy0   0
              0          0         yaw1-yaw0 ]^-1 * (phi0-phi1);
    