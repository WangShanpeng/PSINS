function insserrplot(err, ptype)
% INS square-root error plot for Kalman diag(Pk) output.
%
% Prototype: insserrplot(err, ptype)
% Inputs: err - may be variance of [phi], [phi,dvn], [phi,dvn,dpos],
%                      [phi,dvn,dpos,eb,db], etc. Note: the last colomn is
%                      time tag t.
%         ptype - plot type define
%          
% See also  inserrplot, insplot, kfplot, rvpplot, gpsplot.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/01/2022
    if nargin<2
        inserrplot([sqrt(err(:,1:end-1)), err(:,end)]);
    else
        inserrplot([sqrt(err(:,1:end-1)), err(:,end)], ptype);
    end