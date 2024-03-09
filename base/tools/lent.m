function t = lent(data, ts, t0)
% Set time tag from the length of data.
%
% Prototype: t = lent(data, ts, t0)
% Inputs: data - data in
%         ts - time sampling interval
%         t0  - start time
% Output: t - time tag
%
% See also  appendt, datacut, addclmt.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/11/2023
    if nargin<3, t0=0; end
    if nargin<2, ts=1; end
    t = t0+(1:size(data,1))'*ts;

