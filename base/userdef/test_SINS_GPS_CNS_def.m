function out = test_SINS_GPS_CNS_def(tag, varargin)
% See also  test_SINS_GPS_CNS_216, kfinit, kffk, kfhk, kfplot.
global glv psinsdef
switch tag
	case psinsdef.kfinittag,
        psinsdef.kffk = 21;  psinsdef.kfhk = 219;  psinsdef.kfplot = 219;
        [ins, davp, imuerr, rk, lv, mu] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9+6,1)])^2;  kf.Qk = kf.Qt*ins.nts; kf.Phikk_1 = eye(21);
        kf.Rk = diag(rk)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lv; mu]*1.0)^2;
        kf.Hk = [ zeros(6,3), eye(6), zeros(6,12); ...    % SINS/GPS Hk
                  eye(3), zeros(3,18) ];  kf.Hk(7,7)=1;  % SINS/CNS Hk
        out = kf;
    case psinsdef.kffktag,
        % States: INS(15) + lv(3) + mu^b_s(3)
        ins = setvals(varargin);
        Ft = etm(ins);
        Ft(21,21) = 0;
        out = Ft;
    case psinsdef.kfhktag,
        % Measments: m2rv(Cnb(INS)*Csn(CNS))(3)
        ins = setvals(varargin);
        Hk(1:6,16:18) = [-fins.CW;-fins.MpvCnb];
        Hk(7:9,:) = [eye(3), zeros(3,15), ins.Cnb];  Hk(7,7)=1;
        Hk(8:9,8) = -[ins.eth.cl;ins.eth.sl];
        out = Hk;
    case psinsdef.kfplottag,
        xkpk = setvals(varargin);
        inserrplot(xkpk(:,[1:18,end]));
        subplot(427), hold off; plot(xkpk(:,end), xkpk(:,16:18)/glv.min); xygo('mu');
        inserrplot([sqrt(xkpk(:,19:36)),xkpk(:,end)]);
        subplot(427), hold off; plot(xkpk(:,end), sqrt(xkpk(:,34:36))/glv.min); xygo('mu');
end
