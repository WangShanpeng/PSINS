function t = tgen(ts, T, t0)
% Generate a column time stamp.
%
% Prototype: t = tgen(ts, T, t0)
% Inputs: ts - sampling interval
%         T - end time
%         t0 - start time
% Output: t - time stamp
%
% Example:
%    t = tgen(0.1, 20, 10);  figure, plot(t); grid on;
%
% See also  sortt, adddt, ttest.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/01/2021
    if nargin<2, T=100; end       % t=tgen(ts,T)
    if nargin<1, ts=0.01; end     % t=tgen
    if ts>1, T=100; ts=0.01; end  % t=tgen(T);
    if nargin<3, t0=ts; end
    t = (t0:ts:T)';