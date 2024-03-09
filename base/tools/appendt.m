function data = appendt(data, ts, t0)
% Append time tag to data.
%
% Prototype: data = appendt(data, ts, t0)
% Inputs: data - data in
%         ts - time sampling interval
%         t0  - start time
% Output: data - data out
%
% See also  lent, datacut, addclmt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/06/2022
    if nargin<3, t0=0; end
    if nargin<2, ts=1; end
    data = [data, t0+(1:size(data,1))'*ts];

