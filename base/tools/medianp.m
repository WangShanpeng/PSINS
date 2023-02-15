function [v, idx] = medianp(v, p)
% Find the middle p% elements of vector v.
%
% Prototype: [v, idx] = medianp(v, p)
% Inputs: v - data vector
%         p - p pecentage
%
% See also  smoothol, meann, maxn, cumint.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/12/2022
    if nargin<2, p=0.683; end  % 68.27%, 95.45%, 99.73%
    if p>1, p=min(p/100,1); end
    if size(v,2)>1  % input v for multi-column vectors
        n = size(v,2);
        [vo,idx] = medianp(v(:,1), p);  vo = repmat(vo,1,n);  idx = repmat(idx,1,n);
        for k=2:n, [vo(:,k),idx(:,k)]=medianp(v(:,k), p); end
        v = vo;
        return;
    end
    n = length(v);
    ndel = fix(n*(1-p));
    [v, idx] = sort(v);
    fst=1; lst=n;
    m = v(fix(n/2));
    for k=1:ndel
        if v(lst)-m>m-v(fst), lst=lst-1; else, fst=fst+1; end
    end
    v=v(fst:lst); idx = idx(fst:lst);
