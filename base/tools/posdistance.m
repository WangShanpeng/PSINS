function [pos1, idx] = posdistance(pos, dist)
% Extract postions between two successive points over some distance.
%
% Prototype: [pos1, idx] = posdistance(pos, dist)
% Input: pos - postions
%        dist - distance setting
% Output: pos1 - postions over the distance
%         idx - extract index
%
% See also  vn2dpos.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/09/2020
    idx = ones(length(pos),1);
    kk = 2;
    pos0 = pos(1,1:3)';
    for k=2:length(pos);
        d = norm(pp2vn(pos0, pos(k,1:3)', 1));
        if d>dist, idx(kk) = k; kk = kk+1; pos0 = pos(k,1:3)'; end
    end
    idx(kk:end) = [];
    pos1 = pos(idx,:);