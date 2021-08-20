function [errstd, errmean, errmax, errmin] = avperrstd(avperr, t0, t1, isdisp)
% Calculating AVP error std.
%
% Prototype: errstd = avperrstd(avperr, t0, t1)
% Inputs: avperr - AVP error array
%         t0 - start time point
%         t1  - end time point
% Outputs: errstd - error std.
%          errmean - error mean.
%          errmax - error max.
%          errmin - error min.
%
% See also  avpcmp, avpcmpplot, cep, titlems.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/01/2021
global glv
    if nargin<4, isdisp=1; end
    if nargin<3, t1=avperr(end,end); end
    if nargin<2, t0=avperr(1,end); end
    avperr = datacut(avperr, t0, t1);
    errstd = std(avperr(:,1:9))';
    errmean = mean(avperr(:,1:9))';
    errmax = max(avperr(:,1:9))';
    errmin = min(avperr(:,1:9))';
    if isdisp==1
        err = [errstd, errmean, errmax, errmin];
        err(1:3,:) = err(1:3,:)/glv.min; % arcmin
        %err(4:6,:) m/s;
        err(7:8,:) = err(7:8,:)/glv.sec; % arcsec
        %err(9,:) m;
        disp('   error-std           error-mean         error-max            error-min');
             %   1.456088270321645   4.033029696121787  11.231562149086631  -2.739312623001543
        disp(err);
    end

