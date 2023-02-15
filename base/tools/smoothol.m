function [data, sdata] = smoothol(data, n, iter, isfig)
% smooth outliers.
%
% Prototype: [data, sdata] = smoothol(data, iter, n, isfig)
% Inputs: data - data to processing
%         n - smooth number of points
%         iter - iternation num
%         isfig - figure flag
% Outputs: data - data after processing
%          sdata - smooth data
%
% Example:
%   data = smoothol( abnomaladd(10*sin((1:100)'*0.1)+randn(100,1),0.2,10), 5);
%
% See also  deloutlier, smoothn, medianp, interp1n, abnomaladd, POSSmooth.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/10/2021
    if nargin<4, isfig=1; end
    if nargin<3, iter=3; end
    if nargin<2, n=10; end
    maxn = fix(length(data)/100);
    if isfig, myfig; subplot(211), plot(data,'r'); xygo; end
    data0 = data;
    for k=1:iter
        sdata = smooth(data, n);
        err = data - sdata;
        [~, idx] = sort(err);
        err(idx([1:maxn,end-maxn:end])) = err(idx([1:maxn,end-maxn:end]))/10;
        data = sdata + err;
        if isfig, plot([sdata, data]); end
    end
    if isfig,
        plot(data, 'm');
        subplot(212), plot(data0-sdata); xygo;
    end
    