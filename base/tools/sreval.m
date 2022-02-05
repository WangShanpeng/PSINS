function err = sreval(errs, t, dvel_dyaw, dK_roll, isfig)
% Short-range INS error evaluation. 
%
% Prototype: err = sreval(errs, t, dvel_dyaw)
% Inputs: errs - error source [gyro_drift(deg/h), acc_bias(mg), misalign_ang(deg), init_vel_error(m/s), init_pos_err(m)]
%         t - INS flight time duration
%         dvel_dyaw - INS velocity increament and init_yaw_error [dvel(m/s), dyaw(deg)]
%         dK_roll - roll-gyro scale factor error [roll(round/s), dKroll(ppm)]
% Output: err - short-range INS horizental error
% 
% Example:
%    err = sreval([10, 1, 0.2, 0.3, 10], 60, [100, 1], [2, 100]);
%
% See also  tbinseval, kfstat.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/06/2017, 26/07/2017
global glv
    if nargin<5, isfig=1; end
    errs(6) = 0;
    eb = errs(1)*glv.dph; db = errs(2)*glv.mg; phi = errs(3)*glv.deg; dv = errs(4); dp = errs(5);
    err = [9.8*t^3*eb/6, [db,9.8*phi]*t^2/2, dv*t, dp];
    if exist('dvel_dyaw','var')
        dvel = dvel_dyaw(1); dyaw = dvel_dyaw(2)*glv.deg;
        err = [err, dvel*dyaw*t];
    end
    if exist('dK_roll','var')
        ebK = dK_roll(1)*2*pi*dK_roll(2)/1e6;
        err = [err, 9.8*t^3*ebK/6];
    end
    err = [err, sqrt(err*err'+err(1:3)*err(1:3)')];  % double eb/db/phixy0 err(1:3)
    if isfig==1
        myfig, bar(err); xygo(sprintf('INS error / m @ %.0fs',t));
        if exist('dK_roll','var')
            xtl = {'eb', 'db', 'phixy0', 'dvel0', 'dpos0', 'phiz0*dvel*T', 'wy*dkg', 'Total'};
        elseif exist('vel_dyaw','var')
            xtl = {'eb', 'db', 'phixy0', 'dvel0', 'dpos0', 'phiz0*dvel*T', 'Total'};
        else
            xtl = {'eb', 'db', 'phixy0', 'dvel0', 'dpos0', 'Total'};
        end
        set(gca, 'XTicklabel', xtl);
        xlabel('Error sources');
    end
