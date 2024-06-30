function [x, idx, idxs] = findpeak(x, peak, trough)
% Find peak & trough data.
%
% Prototype:  [x, idx, idxs] = findpeak(x, peak, trough)
% Inputs: x_in - input data
%         peak - num of peak data to be found (or decimal)
%         trough - num of trough data to be found (or decimal)
% Outputs: x_out - out peak & trough data
%          idx,idxs - peak & trough data index found
%
% Example:
%   x = randn(100,3);  [x, idx, idxs] = findpeak(x, 5);
%
% See also  deloutlier, deltrend.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/05/2024
    if nargin<2, peak=10; end
    if nargin<3, trough=0; end
    [m,n] = size(x);
    if peak<1, peak=fix(m*peak); end   % if peak is decimal <1
    if trough<1, trough=fix(m*trough); end
    idx = [];
    for k=1:n
        [xi, I] = sort(x(:,k));
        idxs(:,k) = I([1:trough,end-peak+1:end]);
        idx = union(idx, idxs(:,k));
    end
    myfig, plot(x); hold on; plot(idx, x(idx,:), 'o');
    x = x(idx,:);