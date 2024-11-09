function [data, idx] = inct(data, ts, dts)
% Make data increae by time-tag(INC Time)
%
% Prototype:  [data, idx] = inct(data, ts, dts)
% Inputs: data - input data, the last column is time tag
%         ts - sampling interval, default 1sec
%         dts - sampling interval tolerance, default 1/10*ts
% Outputs: data - output data
%          idx - increasing data index
%
% Example:
%   [t, idx] = inct(([1:10,10,13:15])', 1, 0.1);
%
% See also sortt, reminct, monoinc.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/10/2024
    if nargin<2, ts=1; end
    if nargin<3, dts=ts/10; end
    dt = diffs(data(:,end),1);
    idx = find(dt<ts+dts & dt>ts-dts);
    data = data(idx,:);
    