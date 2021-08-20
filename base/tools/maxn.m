function res = maxn(scr, n, dim)
% Find the n-th max element.
%
% Prototype: res = maxn(scr, n, dim)
% Inputs: scr - data source input to be averaged
%         n - element number to be averaged
%         dim - =1 max along rows, =2 max along columns
%
% See also  meann, sumn, avar.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2020
    if nargin<3
        dim = 1;
    end
    scr = sort(scr, dim, 'descend');
    if dim==1
        res = scr(n,:)';
    else  % dim==2
        res = scr(:,n);
    end
