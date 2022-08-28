function res = nox(scr, clm, val, th)
% To get non-x(in some range) data for specific columns.
%
% Prototype: res = nox(scr, clm, val, th)
% Inputs: scr - data source input
%         clm - column for non-val
%         val,th - some value, threshold
% Output: res - result
%
% See also  no0, nonan, norep, normv.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/07/2022
    if nargin<4, th=eps; end
    if length(clm)>1
        if length(th)==1,  th=repmat(th,length(clm),1);  end
        for k=1:length(clm)
            scr = nox(scr, clm(k), val(k), th(k));
        end
        res = scr;
        return;
    end
    if th==inf,      idx = scr(:,clm)<val;   
    elseif th==-inf, idx = scr(:,clm)>val;
    else             idx = scr(:,clm)>val-th & scr(:,clm)<val+th;
    end
    res = scr(idx,:);
