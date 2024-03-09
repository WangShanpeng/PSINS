function [xk, pk, zk, rk, sk] = igkfplot(kfile, nqr, type, t0)
% KF output plot for C++ class 'CKalman':
% CFileRdWt& CFileRdWt::operator<<(CKalman &kf)
% {
% 	*this<<kf.Xk<<diag(kf.Pk)<<kf.Zk<<kf.Rt<<(double)kf.measflaglog<<kf.kftk;
% 	kf.measflaglog = 0;
% 	return *this;
% }
%
% Prototype: [xk, pk, zk, rk, sk] = igkfplot(kfile, nqr, t0, type)
% Inputs: fname - SINS/GNSS/OD bin file name.
%         nqr - state/measurement dimension, nqr = nq*100+nr
%         flag - plot flag
%         type - 0 for all, 1 for xk/pk, 2 for zk/rk, 3 for state
% See also  igplot, igoplot, kfplot, igload.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/01/2023
global glv
    if nargin<4, t0=0; end
    if nargin<3, type=0; end
    if nqr<1000, nq = fix(nqr/10); nr = nqr-nq*10;
    else,        nq = fix(nqr/100); nr = nqr-nq*100;  end
    [xk, pk, zk, rk, sk] = kffile(kfile, nq, nr, t0);
	t = xk(:,end);
    switch nq
        case 15,
            if nr==6  % CSINSGNSS(15,6)
                if type==0 || type ==1
                    psinstypedef(156);  kfplot(xk,pk, 1:15);
                end
                if type==0 || type ==2  %     // Meas     0-5: SINS/GNSS-dvn,dpos
                    myfig
                    subplot(221), plot(t, zk(:,1:3)); xygo('dV');
                    subplot(222), plot(t, [zk(:,4:5)*glv.Re,zk(:,6)]); xygo('dP');
                    subplot(223), plot(t, sqrt(rk(:,1:3))); xygo('dV');
                    subplot(224), plot(t, [sqrt(rk(:,4:5))*glv.Re,sqrt(rk(:,6))]); xygo('dP');
                end
            end
            if nr==7  % CSINSGNSS(15,7)
                if type==0 || type ==1
                    psinstypedef(156);  kfplot(xk,pk, 1:15);
                end
                if type==0 || type ==2  %     // Meas     0-5: SINS/GNSS-dvn,dpos
                    myfig
                    subplot(231), plot(t, zk(:,1:3)); xygo('dV');
                    subplot(232), plot(t, [zk(:,4:5)*glv.Re,zk(:,6)]); xygo('dP');
                    subplot(233), plot(t, zk(:,7)/glv.deg); xygo('dYaw / \circ');
                    subplot(234), plot(t, sqrt(rk(:,1:3))); xygo('dV');
                    subplot(235), plot(t, [sqrt(rk(:,4:5))*glv.Re,sqrt(rk(:,6))]); xygo('dP');
                    subplot(236), plot(t, sqrt(rk(:,7))/glv.deg); xygo('dYaw / \circ');
                end
            end
        case 16,
            if nr==6  % CSINSGNSS(16,6)
                if type==0 || type ==1
                    psinstypedef(166);  kfplot(xk,pk, 1:16);
                end
                if type==0 || type ==2  %     // Meas     0-5: SINS/GNSS-dvn,dpos
                    myfig
                    subplot(221), plot(t, zk(:,1:3)); xygo('dV');
                    subplot(222), plot(t, [zk(:,4:5)*glv.Re,zk(:,6)]); xygo('dP');
                    subplot(223), plot(t, sqrt(rk(:,1:3))); xygo('dV');
                    subplot(224), plot(t, [sqrt(rk(:,4:5))*glv.Re,sqrt(rk(:,6))]); xygo('dP');
                end
            end
        case 18,
            if nr==17  % CAutoDrive
                if type==0 || type ==1
                    psinstypedef(156);  kfplot(xk,pk, 1:15); xpplot(xk,pk,16:18,'kappa');
                end
                if type==0 || type ==2  %     // Meas     0-5: SINS/GNSS-dvn,dpos; 6-8: SINS/OD-dvn; 9-11: ZUPT; 12-14: NHC; 15: SINS/GNSS-dyaw; 16: WzHold
                    myfig
                    subplot(321), plot(t, zk(:,1:3)); xygo('dV');
                    subplot(322), plot(t, [zk(:,4:5)*glv.Re,zk(:,6)]); xygo('dP');
                    subplot(323), plot(t, zk(:,7:9)); xygo('dvnOD / m/s');
                    subplot(324), plot(t, zk(:,10:15)); xygo('dvnZUPT/NHC / m/s');
                    subplot(325), plot(t, zk(:,16)/glv.deg); xygo('dyaw');
                    subplot(326), plot(t, zk(:,17)/glv.dph); xygo('WzHold / (\circ/h)');
                    myfig
                    subplot(321), plot(t, sqrt(rk(:,1:3))); xygo('dV');
                    subplot(322), plot(t, [sqrt(rk(:,4:5))*glv.Re,sqrt(rk(:,6))]); xygo('dP');
                    subplot(323), plot(t, sqrt(rk(:,7:9))); xygo('dvnOD / m/s');
                    subplot(324), plot(t, sqrt(rk(:,10:15))); xygo('dvnZUPT/NHC / m/s');
                    subplot(325), plot(t, sqrt(rk(:,16))/glv.deg); xygo('dyaw');
                    subplot(326), plot(t, sqrt(rk(:,17))/glv.dph); xygo('WzHold / (\circ/h)');
                end
            end
        case 27,
            if nr==10  % CSGOClbt
                if type==0 || type ==1
                    psinstypedef(186);
                    kfplot(xk,pk, 1:18);
                    myfig;
                    subplot(321), plot(t, xk(:,[19,21])/glv.deg); xygo('dP,dY / \circ'); title('Pitch/Yaw Install Angle')
                    subplot(322), plot(t, xk(:,20)*1000); xygo('dKod / 0.1%'); title('OD Scale Error')
                    subplot(323), plot(t, xk(:,22:24)); xygo('lever'); title('Lever arm')
                    subplot(324), plot(t, xk(:,25)); xygo('dT');
                    subplot(325), plot(t, xk(:,26)/glv.deg); xygo('phiU');
                    subplot(326), plot(t, xk(:,27)/glv.ppm); xygo('dKzz'); title('Scale Gyro')
                    myfig;
                    subplot(321), plot(t, sqrt(pk(:,[19,21]))/glv.deg); xygo('dP,dY / \circ'); title('Pitch/Yaw Install Angle')
                    subplot(322), plot(t, sqrt(pk(:,20))*1000); xygo('dKod / 0.1%'); title('OD Scale Error')
                    subplot(323), plot(t, sqrt(pk(:,22:24))); xygo('lever'); title('Lever arm')
                    subplot(324), plot(t, sqrt(pk(:,25))); xygo('dT');
                    subplot(325), plot(t, sqrt(pk(:,26))/glv.deg); xygo('phiU');
                    subplot(326), plot(t, sqrt(pk(:,27))/glv.ppm); xygo('dKzz'); title('Scale Gyro')
                end
                if type==0 || type ==2  %     // Meas     0-5: SINS/GNSS-dvn,dpos; 6-8: SINS/OD-dvn; 9-11: ZUPT; 12-14: NHC; 15: SINS/GNSS-dyaw; 16: WzHold
                    myfig
                    subplot(221), plot(t, zk(:,1:3)); xygo('dV');
                    subplot(222), plot(t, [zk(:,4:5)*glv.Re,zk(:,6)]); xygo('dP');
                    subplot(223), plot(t, zk(:,7:9)); xygo('dvnOD / m/s');
                    subplot(224), plot(t, zk(:,10)/glv.deg); xygo('dyaw');
                    myfig
                    subplot(221), plot(t, sqrt(rk(:,1:3))); xygo('dV');
                    subplot(222), plot(t, [sqrt(rk(:,4:5))*glv.Re,sqrt(rk(:,6))]); xygo('dP');
                    subplot(223), plot(t, sqrt(rk(:,7:9))); xygo('dvnOD / m/s');
                    subplot(224), plot(t, sqrt(rk(:,10))/glv.deg); xygo('dyaw');
                end
            end
        case {37,46}  % CSysClbt
            myfig
            t = xk(:,end)/tscaleget;
            subplot(331), plot(t, xk(:,1:3)/glv.min); xygo('phi');
            subplot(332), plot(t, xk(:,4:6)); xygo('dV')
            subplot(333), plot(t, xk(:,7:9)/glv.dph); xygo('eb');
            subplot(334), plot(t, xk(:,10:12)/glv.ug); xygo('db');
            subplot(335), plot(t, xk(:,13:4:21)/glv.ppm); xygo('dKii');
                hold on,  plot(t, xk(:,[22,25,27])/glv.ppm, '--');
            subplot(336), plot(t, xk(:,[14:16,18:20])/glv.sec); xygo('dKij');
                hold on,  plot(t, xk(:,[23,24,26])/glv.sec, '--');
%             subplot(337), plot(t, xk(:,28:30)/glv.ugpg2); xygo('Ka2');
            subplot(337), plot(t, xk(:,28:30)/glv.ppm); xygo('Kapn / ppm');
            subplot(338), plot(t, xk(:,31:33)*100); xygo('lever arm / cm');
                hold on,  plot(t, xk(:,34:36)*100,'--');
            subplot(339), plot(t, xk(:,37)*1000); xygo('\tau_{GA} / ms');
            myfig
            t = pk(:,end)/tscaleget; sp=sqrt(pk(:,1:end-1));
            subplot(331), plot(t, sp(:,1:3)/glv.min); xygo('phi');
            subplot(332), plot(t, sp(:,4:6)); xygo('dV')
            subplot(333), plot(t, sp(:,7:9)/glv.dph); xygo('eb');
            subplot(334), plot(t, sp(:,10:12)/glv.ug); xygo('db');
            subplot(335), plot(t, sp(:,13:4:21)/glv.ppm); xygo('dKii');
                hold on,  plot(t, sp(:,[22,25,27])/glv.ppm, '--');
            subplot(336), plot(t, sp(:,[14:16,18:20])/glv.sec); xygo('dKij');
                hold on,  plot(t, sp(:,[23,24,26])/glv.sec, '--');
%             subplot(337), plot(t, sp(:,28:30)/glv.ugpg2); xygo('Ka2');
            subplot(337), plot(t, sp(:,28:30)/glv.ppm); xygo('Kapn / ppm');
            subplot(338), plot(t, sp(:,31:33)*100); xygo('lever arm / cm');
                hold on,  plot(t, sp(:,34:36)*100,'--');
            subplot(339), plot(t, sp(:,37)*1000); xygo('\tau_{GA} / ms');
            if nq==46
                myfig
                subplot(321), plot(t, xk(:,38:40)/glv.secpg); xygo('Xxyz / (\prime\prime/g)')
                subplot(323), plot(t, xk(:,41:43)/glv.secpg); xygo('Yxyz / (\prime\prime/g)')
                subplot(325), plot(t, xk(:,44:46)/glv.secpg); xygo('Zxyz / (\prime\prime/g)')
                subplot(322), plot(t, sp(:,38:40)/glv.secpg); xygo('Xxyz / (\prime\prime/g)')
                subplot(324), plot(t, sp(:,41:43)/glv.secpg); xygo('Yxyz / (\prime\prime/g)')
                subplot(326), plot(t, sp(:,44:46)/glv.secpg); xygo('Zxyz / (\prime\prime/g)')
            end
    end
    if type==0 || type ==3
        stateplot(sk);
    end

