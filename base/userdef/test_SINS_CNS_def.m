function out = test_SINS_CNS_def(tag, varargin)
% See also  test_SINS_CNS_184, kfinit, kffk, kfhk, kfplot.
global glv psinsdef
switch tag
	case psinsdef.kfinittag,
        psinsdef.kffk = 18;  psinsdef.kfhk = 184;  psinsdef.kfplot = 184;
        [ins, davp, imuerr, rk, mu] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9+3,1)])^2;
        kf.Rk = diag(rk)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db; mu]*1.0)^2;
        kf.Hk = zeros(4,18);
        out = kf;
    case psinsdef.kffktag,
        % States: INS(15) + mu^b_s(3)
    case psinsdef.kfhktag,
        % Measments: m2rv(Cnb(INS)*Csn(CNS))(3) + dHgt(1)
        ins = setvals(varargin);
        Hk = [eye(3), zeros(3,12), ins.Cnb; zeros(1,18)];  Hk(1,7)=1;  Hk(4,9)=1;
        Hk(2:3,8)=-[ins.eth.cl;ins.eth.sl];
        out = Hk;
    case psinsdef.kfplottag,
        xkpk = setvals(varargin);
        inserrplot(xkpk(:,[1:18,end]));
        subplot(427), hold off; plot(xkpk(:,end), xkpk(:,16:18)/glv.min); xygo('mu');
        inserrplot([sqrt(xkpk(:,19:36)),xkpk(:,end)]);
        subplot(427), hold off; plot(xkpk(:,end), sqrt(xkpk(:,34:36))/glv.min); xygo('mu');
end
