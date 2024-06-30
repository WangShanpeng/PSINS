function [gsen, gsenu, err] = lsgsen(imu, dir0, pos0, yaw0, isfig)
% SIMU gyro bias for senstivity to acceleration-forces using Least-square method.
%
% Prototype: [gsen, gsenu, err] = lsgsen(wbib, dir0, yaw0, isfig)
% Inputs: imu - =[wbib,fbsf], wbib in rad/s, fbsf in m/s^2
%         dir0 - SIMU direction at the first epoch, w.r.t. R-F-U
%         pos0 - test position
%         yaw0 - relative yaw at the fist epoch
%         isfig - figure flag
% Outputs: gsen - g-senstivity.
%          gsenu - [bias in dph, & g-sen in dph/g]
%          err - LS residual error
%
% See also  lsclbt, cumwie, dir2att.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/04/2024
    global glv
    if nargin<5, isfig=0; end
    if nargin<4, yaw0=0; end
    if nargin<3, pos0=[0;0;0]; end
    [wnie, g] = wnieg(pos0);
    y = imu(:,1:3);  A = zeros(length(y),4);  wbiek = y;
    for k=1:size(imu,1)
        if size(dir0,1)>1, att=dir2att(dir0(k,:),yaw0); else, att=[0;0;yaw0]; end
        wbie = a2mat(att)'*wnie;  wbiek(k,:) = wbie';
        y(k,:) = imu(k,1:3);
        A(k,:) = [1, imu(k,4:6)];
    end
    for k=1:3
        yk = y(:,k)-wbiek(:,k);
        gsen(:,k) = lscov(A, yk);
        err(:,k) = yk-A*gsen(:,k);
    end
    if isfig
        myfig, xyz='XYZ';
        for k=1:3, subplot(3,1,k), plot([y(:,k)-gsen(1,k), wbiek(:,k), err(:,k)]/glv.dph); xygo('k',xyz(k)); end
        subplot(311), legend('eb-eb0', 'wbie', 'error');
    end
    gsenu = [gsen(1,:)/glv.dph; gsen(2:4,:)/glv.dphpg];
    err = err/glv.dph;
