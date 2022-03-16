function [xt, pt] = xpclm(xpt, clm)
% Extract xt,pt from KF result xpt array.
%
% Prototype: [xt, pt] = xpclm(xpt, clm)
% Inputs: xpt - KF result xpt array
%         clm - designate column
% Output: xt,pt - designate x,p data column
%          
% See also  kfplot, xpplot.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/03/2022
    if ischar(clm)
        if strcmp(clm,'avp'), clm=1:9;
        elseif strcmp(clm,'avped'), clm=1:15; end
    end
    if length(clm)==1, clm=1:clm; end
    n2 = (size(xpt,2)-1)/2;
    if nargout==2;
        xt = xpt(:, [n2; end]);
        pt = xpt(:, [n2+clm(:); end]);
    elseif nargout==1
        xt = xpt(:, [clm(:); n2+clm(:); end]);  % xpt = xpclm(xpt, clm)
    end