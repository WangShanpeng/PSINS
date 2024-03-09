function data = diffs(data, be)
% Data derivative, the output keep same length with input (diff Same).
%
% Prototype: res = sumn(scr, n, dim)
% Inputs: data - data input
%         be - begin or end extrapolation, =1 for begin, =-1 for end, =0 for mid.
% Output: data - derivative data output, keep same length with input
%
% See also  meann, maxn, cumint, diff.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/06/2021
    if nargin<2, be = 0;  end
    data = diff(data);
    if be==1
        data = [2*data(1,:)-data(2,:); data];
    elseif be==-1
        data = [data; 2*data(end,:)-data(end-1,:)];
    else
        d1 = [2*data(1,:)-data(2,:); data];
        d2 = [data; 2*data(end,:)-data(end-1,:)];
        data = (d1+d2)/2;
    end
