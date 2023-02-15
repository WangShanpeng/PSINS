function res = subavp(avped, substr)
% Extract data from a AVPED data array.
%
% Prototype: res = subavp(avped, substr)
% Inputs: avped - AVPED data array
%         substr - sub setting string
% Output: res - result
%
% See also: avpidx, insplot, avpcmpplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/09/2021
    if nargin<2, substr='avp'; end
    idx = [];
    for k=1:length(substr)
        switch substr(k)
            case 'a', idx = [idx,1:3];
            case 'v', idx = [idx,4:6];
            case 'p', idx = [idx,7:9];
            case 'e', idx = [idx,10:12];
            case 'd', idx = [idx,13:15];
        end
    end
    res = avped(:,[sort(idx),end]);
    