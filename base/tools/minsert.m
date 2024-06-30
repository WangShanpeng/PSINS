function m = minsert(m, idx, v)
% Matrix insert with specified vector.
%
% Prototype: m = minsert(m, idx, v)
% Inputs: m - data input
%         idx - index to add
%         v - specified vector
% Output; m - data output
%
% Example
%    m = minsert(randn(10,6), [2,3,5], [1,2,3]);
%
% See also  vinsert, getat, sortt, tshift, delbias, scalet, addclmt.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/01/2024
    if nargin<3, v=0*idx(:)'; end
    if size(v,1)==1, v=repmat(v,size(m,1),1); end
    idx = sort(idx);
    for k=1:length(idx)
        idxk = idx(k);
        if idx==1
            m = [v(:,k), m];
        else
            m = [m(:,1:idxk-1), v(:,k), m(:,idxk:end)];
        end
    end