function [att, att0, eN, dU, av] = alignpe(imu, pos, eU, i0t, isfig)
% SINS initial alignment uses parameter estimating.
%
% Prototype: [att, att0, eN, dU, av] = alignpe(imu, pos, eU, i0t, isfig)
% Inputs: imu - SIMU data
%         pos - initial position
%         eU - upward gyro drift
%         i0t - interval for aligni0
%         isfig - figure flag
% Outputs: att, attitude align results Euler angles, att0 is the start attitude
%          eN, dU - northward gyro drift, upward acc bias estimate
%          av - [att,vn,t] open-loop navagtion result
%
% Example
%   imu = imustatic([[0;0;0]*glv.deg; glv.pos0], 0.1, 100, imuerrset(0.1, 0));
%   [att, att0, eN, dU, av] = alignpe(imu, glv.pos0, 0.1*glv.dph, 30, 1); % insplot(av,'av');
%
% See also  aligni0, alignsb, alignpetp, vn2phi.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/07/2024
global glv
    tmpfig = 0;
    if nargin<5,  isfig = 1;  end
    if nargin<4,  i0t = 30;  end
    if nargin<3,  eU = 0;  end
    [nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    [atti0, res] = aligni0(imu(1:fix(i0t/ts),:), pos, tmpfig);
    att0 = res.att0;  eN=0;  dU=0;
    nextlinestyle(-1);
    for iter=1:4
        av = inspure(imu, [att0;pos], 'O', tmpfig);
        [phi, eNi, dUi] = vn2phi(av(:,[4:6,end]), pos, [], tmpfig);
        att0 = adelphi(att0,phi(1,1:3)');
        if isfig==1 && iter>1
            eb = a2mat(att0)'*[0;eNi;eU];  eN=eN+eNi;  eU=0;  % cumulate eN, but use eU only once
            db = a2mat(att0)'*[0;0;dUi];  dU=dU+dUi;
            edbts = [eb; db]*ts;
            for k=1:6, imu(:,k) = imu(:,k)-edbts(k); end  % del bias
            if iter==2, myfig; end
            subplot(321), plot(phi(:,end), phi(:,1)/glv.sec, nextlinestyle(1)), xygo('phiE');
            subplot(323), plot(phi(:,end), phi(:,2)/glv.sec, nextlinestyle(0)), xygo('phiN');
            subplot(325), plot(phi(:,end), phi(:,3)/glv.min, nextlinestyle(0)), xygo('phiU');
            subplot(322), plot(av(:,end), av(:,4), nextlinestyle(0)), xygo('VE');
            subplot(324), plot(av(:,end), av(:,5), nextlinestyle(0)), xygo('VN');
            subplot(326), plot(av(:,end), av(:,6), nextlinestyle(0)), xygo('VU');  title(['\nabla_U = ',sprintf( '%.2f (ug)', dU/glv.ug)]);
        end
    end
    av = av(:,[1:6,end]);  att = av(end,1:3)';

