function x1 = replaceby(x1, x2, clm, method)
% Replace data x1 with specific column data in data x2.
%
% Prototype: x1 = replaceby(x1, x2, clm, method)
% Inputs: x1 - [data1,t1]
%         x2 - [data2,t2]
%         clm - specific column
%         method - interpolation method
% Output: x1 - new data x1 output
%
% Example:
%    x1 = [randn(10,2),(1:10)'];  x2 = [randn(20,2),(5.1:25)']; 
%    x1new = replaceby(x1, x2);
%    myfig; plot(x1(:,end),x1(:,1:2), x2(:,end),x2(:,1:2), x1new(:,end),x1new(:,1:2)); grid on;
%    
% See also interp1n.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/06/2021
    if nargin<4, method = 'linear'; end
    if nargin<3, clm=1:size(x2,2)-1; end
    if isempty(clm), clm=1:size(x2,2)-1; end  % x1 = replaceby(x1, x2, [], method);
    ts = max(x1(1,end),x2(1,end)); te = min(x1(end,end),x2(end,end));
    ns = find(x1(:,end)>ts,1,'first'); ne = find(x1(:,end)<te,1,'last');
    t = x1(ns:ne,end);
    for k=1:length(clm)
        x1(ns:ne,clm(k)) = interp1(x2(:,end), x2(:,clm(k)), t, method);  % x2(:,clm(k)) -> x1(:,clm(k))
    end
