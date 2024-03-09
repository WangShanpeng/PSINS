function [Kg, eb, Ka, db] = lsclbt(wbib, yw, fbsf, yf)
% SIMU calibration using Least-square method.
%
% Prototype: [Kg, eb, Ka, db] = lsclbt(wbib, yw, fbsf, yf)
% Inputs: wbib - SIMU gyro angular rate,
%         yw - turntable angular rate,
%         fbsf - SIMU acc specific force,
%         yf - turntable specific force.
% Outputs: Kg, eb, Ka, db - calibration results, or Kg=clbt structure.
%
% See also  sysclbt, imuclbt, clbtdiff.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/01/2020
    Xw = [];
    for k=1:size(wbib,1)
        Xw(3*(k-1)+1:3*k,:) = [wbib(k,:),zeros(1,6),-1,0,0; zeros(1,3),wbib(k,:),zeros(1,3),0,-1,0; zeros(1,6),wbib(k,:),0,0,-1];
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
