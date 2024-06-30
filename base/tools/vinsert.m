function v = vinsert(v, idx, ele)
% Vector insert with specified element.
%
% Prototype: v = vinsert(v, idx, ele)
% Inputs: v - data input
%         idx - index to add
%         ele - specified element
% Output; v - data output
%
% Example
%    v = vinsert(randn(10,1), [5,6,9], 10);  plotn(v);
%
% See also  minsert, getat, sortt, tshift, delbias, scalet, addclmt.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/10/2023
    if nargin<3, ele=0; end
    idx = sort(idx);
    for k=1:length(idx)
        idxk = idx(k);
        if idx==1
            v = [ele; v];
        else
            v = [v(1:idxk-1); ele; v(idxk:end)];
        end
    end