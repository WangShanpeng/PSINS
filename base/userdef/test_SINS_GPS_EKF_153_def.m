function out = test_SINS_GPS_EKF_153_def(tag, varargin)
% See also  test_SINS_GPS_UKF_153.
global glv psinsdef
switch tag
	case psinsdef.kfinittag,
        [ins, davp, imuerr, rk] = setvals(varargin);
        kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(9,1)])^2;
        kf.Rk = diag(rk)^2;
        kf.Pxk = diag([davp; imuerr.eb; imuerr.db]*1.0)^2;
        kf.fx = @largephiu15ekf;
        kf.Hk = [zeros(3,6), eye(3), zeros(3,6)];
        out = kf;
    case psinsdef.kffktag,
    case psinsdef.kfhktag,
    case psinsdef.kfplottag,
        [xkpk, avperr, imuerr] = setvals(varargin);
        psinstypedef(153);
        kfplot(xkpk, avperr, imuerr);
        psinstypedef('test_SINS_GPS_UKF_153_def');
end
