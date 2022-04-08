function out = test_SINS_GPS_PF_153_def(tag, varargin)
% See also  test_SINS_GPS_PF_153.
global glv psinsdef
switch tag
	case psinsdef.kfinittag,
        [ins, davp, imuerr, rk, N] = setvals(varargin);
        Qt = diag([imuerr.web; imuerr.wdb; zeros(9,1)])^2;
        Rk = diag(rk)^2;
        Pxk = diag([davp; imuerr.eb; imuerr.db]*1.0)^2;
        pf = pfinit(zeros(15,1), Pxk, Qt*ins.nts, Rk, N);
        pf.fx = @largephiu15ukf;
        pf.Hk = [zeros(3,6), eye(3), zeros(3,6)];
        out = pf;
    case psinsdef.kffktag,
    case psinsdef.kfhktag,
    case psinsdef.kfplottag,
        [xkpk, avperr, imuerr] = setvals(varargin);
        psinstypedef(153);
        kfplot(xkpk, avperr, imuerr);
        psinstypedef('test_SINS_GPS_PF_153_def');  % restore
end
