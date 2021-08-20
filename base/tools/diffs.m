function data = diffs(data, be)
% Data derivative, the output keep same length with input (diff Same).
%
% Prototype: res = sumn(scr, n, dim)
% Inputs: data - data input
%         be - begin or end extrapolation, =1 for begin, else for end.
% Output: data - derivative data output, keep same length with input
%
% See also  meann, maxn, cumint, diff.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/06/2021
    if nargin<2, be = 1;  end
    data = diff(data);
    if be==1
        data = [2*data(1,:)-data(2,:); data];
    else
        data = [data; 2*data(end,:)-data(end-1,:)];
    end
