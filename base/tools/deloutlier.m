function [x, outlierIdx] = deloutlier(x, bound, clm, nSmooth)
% Delete outlier.
%
% Prototype:  [x, outlierIdx] = deloutlier(x, bound, clm, nSmooth)
% Inputs: x_in - input data with bias
%         bound - outlier lower&upper bound
%         clm - data column to find outlier
%         nSmooth - n points to find outlier
% Outputs: x_out - output data with no bias
%          outlierIdx - outlier index found
%
% Example:
%   x = addclmt(abnomaladd(10*sin((1:100)'*0.1)+randn(100,1),0.2,10));
%   y = deloutlier(x, 5, 1, 5);  myfig; plot(x(:,2),x(:,1), y(:,2),y(:,1));
%
% See also  smoothol, deltrend, imudeldrift, findpeak.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/10/2022 
    if nargin<4, nSmooth=1; end
    if nargin<3, clm=1; end
    if length(bound)<2, upBound=abs(bound); lowBound=-upBound; 
    else, upBound=max(bound); lowBound=min(bound); end
    xx = sum(x(:,clm),2);  xx = xx-median(xx);
    if nSmooth==1
        outlierIdx = xx>upBound | xx<lowBound;
        x = x(~outlierIdx,:);
        outlierIdx = find(outlierIdx);
    else
        data = smoothol(xx, min(nSmooth,5), 2, 0);
        [~, outlierIdx] = deloutlier(xx-data, [lowBound, upBound], 1, 1);
        x(outlierIdx,:) = [];
    end