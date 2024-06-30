function [xkpk, kfs, trj] = tbinseval(trj, avperr, imuerr, isfig)
% Trajectory-based INS error evaluation. 
%
% Prototype: [xkpk, kfs] = tbinseval(trj, avperr, imuerr, isfig)
% Inputs: trj - trajectory struct includes IMU & AVP info
%         avperr - init AVP err for KF P0 setting
%         imuerr - for KF P0/Qt setting
%         isfig - figure flag
% Outputs: xkpk, kfs - xkpk, kf statistic
% 
% See also  sreval, kfstat, kfupdate, trjsimu, imuerrset, avperrset.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/11/2021
global glv
    if nargin<4, isfig=1; end
    psinstypedef(331);
    if isstruct(trj)
        if ~isfield(trj,'avp0'), trj.avp0=trj.avp(1,1:9)'; end;
        imu=trj.imu; avp0=trj.avp0; avp=trj.avp;
    else
        [imu, avp0, avp] = ap2imu(trj(:,[1:3,end-3:end]));
    end
    clear trj; trj.imu = imu; trj.avp = avp; trj.avp0 = avp0;
    if isempty(avperr), avperr = avperrset([0.1;0.1;1], 0.01, 0.1);  end
    if isempty(imuerr), imuerr = imuerrset(0.01,30,0.001,3,  0,1,0,1,  3,3, 3,3, 3);  end
    aierr = [avperr; imuerr.eb; imuerr.db; imuerr.dKga; imuerr.Ka2; imuerr.web; imuerr.wdb];  % basic error 39x1
    ts = diff(imu(1:2,end));
    ins = insinit(avp0, ts);  ins.nts=ts;
    kf = kfinit(ins, avperr, imuerr, 0);
    kfs = kfstat([], kf);
    len = min(length(imu),length(avp));
    xkpk = prealloc(len, 2*kf.n+1);
    ki = timebar(1, len, 'Trajectory-based INS error evaluation.');
    for k=1:len
        ins = insupdate(ins, imu(k,1:6)); ins.vn=avp(k,4:6)'; ins.pos=avp(k,7:9)';
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        kfs = kfstat(kfs, kf, 'T');
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); imu(k,end)]';
        ki = timebar;
    end
    kfs = kfstat(kfs);
    Pii = diag(kfs.Pk);  Pii(1:3) = Pii(1:3)/glv.min^2;  Pii(7:8) = Pii(7:8).*[ins.eth.RMh;ins.eth.clRNh].^2;
    pqt = [kfs.p, kfs.q(:,1:6), Pii];
    for k=1:length(pqt)-1, pqt(1:9,k)=pqt(1:9,k).*pqt(1:9,end);  end  % percent to abs
    pqt=sqrt(pqt(1:9,:));
    kfs.pqt = pqt(1:9,:);  kfs.aierr = aierr;  % save to kfs
    if isfig==1
        inserrplot([sqrt(xkpk(:,34:45)),xkpk(:,end)]);
        myfig,
        n = 40;
        subplot(311), plot(pqt(1:3,:)','-.o','linewidth',2); xlim([1,n]); grid on; ylabel('\phi / (\prime)'); legend('\phi_E', '\phi_N', '\phi_U', 'Location','Best');
        xtl = {'phiE/N/U', 'dVE/N/U', 'dLat/Lon/Hgt', 'ebx/y/z', 'dbx/y/z', 'dkgx/y/zx', 'dkgx/y/zy', 'dkgx/y/zz', 'dkax/y/zx', 'dkay/zy,zz', 'Ka2x/y/z', 'wgx/y/z', 'wax/y/z', 'Total'};
        set(gca, 'xtick', [2:3:n,n], 'XTicklabel', xtl);
        subplot(312), plot(pqt(4:6,:)','-.o','linewidth',2); xlim([1,n]); grid on; ylabel('\deltaV / m/s'); legend('\deltaV_E', '\deltaV_N', '\deltaV_U', 'Location','Best');
        set(gca, 'xtick', [2:3:n,n], 'XTicklabel', xtl);
        subplot(313), plot(pqt(7:9,:)','-.o','linewidth',2); xlim([1,n]); grid on; ylabel('\deltaP / m'); legend('\deltaL', '\delta\lambda', '\deltaH', 'Location','Best');
        title(sprintf('T = %.3f (s);  Distance = %.3f (m);  CEP50 = %.3f (m)', avp(end,end)-avp(1,end), norm(pp2vn(avp(end,7:9)',avp(1,7:9)',1)) , sum(pqt(7:8,end))*0.59));
        set(gca, 'xtick', [2:3:n,n], 'XTicklabel', xtl);
%         xtl = {'\phi_E', '\phi_N', '\phi_U', '\deltaV_E', '\deltaV_N', '\deltaV_U', '\deltaL', '\delta\lambda', '\deltaH', ...
%             '\epsilon_X', '\epsilon_Y', '\epsilon_Z', '\nabla_X', '\nabla_Y', '\nabla_Z', ...
%             '\deltak_{gxx}', '\deltak_{gxy}', '\deltak_{gxz}', '\deltak_{gyx}', '\deltak_{gyy}', '\deltak_{gyz}', '\deltak_{gzx}', '\deltak_{gzy}', '\deltak_{gzz}', ...
%             '\deltak_{axx}', '\deltak_{ayx}', '\deltak_{ayy}', '\deltak_{azx}', '\deltak_{azy}', '\deltak_{azz}', ...
%             'W_{gx}', 'W_{gy}', 'W_{gz}', 'W_{ax}', 'W_{ay}', 'W_{az}'};
%         set(gca, 'xtick', 1:36, 'XTicklabel', xtl);
    end