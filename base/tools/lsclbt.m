function [Kg, eb, Ka, db] = lsclbt(wbib, yw, fbsf, yf)
% SIMU calibration using Least-square method.
%
% Prototype: [Kg, eb, Ka, db] = lsclbt(wbib, yw, fbsf, yf)
% Inputs: wbib - SIMU gyro angular rate in rad/s, [if size(wbib,2)==4, then wbib is
%                angular increment and the last column is sampling time span]
%         yw - turntable angular rate [or angular increment],
%         fbsf - SIMU acc specific force in m/s^2,
%         yf - turntable specific force.
% Outputs: Kg, eb, Ka, db - calibration results, or Kg=clbt structure.
%                NOTE: gyro bias 'eb' may be inaccurate under rotation condition
%
% See also  sysclbt, imuclbt, clbtdiff, qrclbt, Ka2dKphi, agdir, cumwie, lsgsen.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/01/2020
    Xw = [];
	if size(wbib,2)==3, wbib(:,4)=1; end
    for k=1:size(wbib,1)
        Xw(3*(k-1)+1:3*k,:) = [ wbib(k,1:3),zeros(1,6),           -wbib(k,4),0,0;
                                zeros(1,3),wbib(k,1:3),zeros(1,3), 0,-wbib(k,4),0;
                                zeros(1,6),wbib(k,1:3),            0,0,-wbib(k,4) ];
        Yw(3*(k-1)+1:3*k,1) = yw(k,:)';
    end
    Kw = (Xw'*Xw)^-1*Xw'*Yw;
    Kg = reshape(Kw(1:9),3,3)';
    eb = Kw(10:12);
    if nargin>2  % for acc calibration
        [Ka, db] = lsclbt(fbsf, yf);
    end
    if nargout==1
        clbt.Kg = Kg;  clbt.eb = eb;  clbt.Ka = Ka;  clbt.db = db;
        Kg = clbt;
    end
