function [da, p1, p2] = kaafit(data, t1, t2, isfig)
% Two piecewise linear fit.
%
% Prototype: [da, p1, p2] = kaafit(data, t1, t2, isfig)
% Inputs: data - two-column input data = [data, t]
%         t1, t2 - [start,t1] & [t2,end] for piecewise time tag
% Outputs: da, p1, p2 - linear fit coefficients p1 & p2, where
%                       p1 = k*t+a1, p2 = k*t+a2, da = a2-a1
%
% Example
%    t1=1:10; t2=21:30; t=[t1,t2]'; 
%    [da, p1, p2] = kaafit([[t1, t2+10]'+randn(20,1), t], 10, 20, 1);
%    
% See also  smoothol, polyfit.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/08/2023
    if size(data,2)==1, data=[data,(1:length(data))']; end
    idx1 = data(:,end)<t1;  t1 = data(idx1,end);
    y1 = data(idx1,1);
    A = [t1, ones(size(y1)), zeros(size(y1))];
    idx2 = data(:,end)>t2;  t2 = data(idx2,end);   
    y2 = data(idx2,1);   y = [y1; y2];
    A = [A; [t2, zeros(size(y2)), ones(size(y2))]];
    x = lscov(A, y);
    p1 = x(1:2); p2 = x([1,3]);
    da = p2(2)-p1(2);
    if nargin<4, isfig=1; end
    if isfig
        idx = idx1|idx2;
        t = data(idx,end);  t2 = t([1,end]);
        myfig, plot(t, data(idx,1), t2, polyval(p1,t2), t2,polyval(p2,t2)); xygo('val');
        title(sprintf('a2-a1 = %f', da));
    end