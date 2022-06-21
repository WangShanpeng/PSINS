function v1v5 = v5double(v5v1)
% Translate 5-column std vector within [0,990] to 1-column double vector,
% or vice versa.
%
% Prototype: v1v5 = v5double(v5v1)
% Input: v5v1 - 5-column std vector within [0,990], or 1-column vector
% Output: v1v5 - 1-column double vector, or 5-column std vector within [0,990],
%                NOTE: the LSB may be inaccurate
% Example:
%    v5=[100;2000;-30;3000;100]; v1=v5double(v5); [v5,v5double(v1)]
%    v5=round(abs(randn(10,5))*1e2); v1=v5double(v5); myfig(v5); plot(v5double(v1),'-.')
%
% See also  v2double, v3double.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/05/2022
    [m,n] = size(v5v1);
    if m<=5&&n==1, v5v1=v5v1'; [m,n] = size(v5v1); end
    if n>1  % v5->v1
        if n<5, v5v1 = [v5v1, ones(m,5-n)]; end
        v5v1(v5v1>990) = 991;  v5v1(v5v1<=0) = eps;
        v1v5 = round(v5v1(:,1)) + round(v5v1(:,2))/1e3 + round(v5v1(:,3))/1e6 + round(v5v1(:,4))/1e9 + round(v5v1(:,5))/1e12;
    else % v1->v5
        v1v5(:,1) = floor(v5v1+eps);  v5v1=(v5v1-v1v5(:,1))*1e3;
        v1v5(:,2) = floor(v5v1+eps);  v5v1=(v5v1-v1v5(:,2))*1e3;
        v1v5(:,3) = floor(v5v1+eps);  v5v1=(v5v1-v1v5(:,3))*1e3;
        v1v5(:,4) = floor(v5v1+eps);  v5v1=(v5v1-v1v5(:,4))*1e3;
        v1v5(:,5) = floor(v5v1+eps);
        if size(v1v5,1)==1, v1v5=v1v5'; end
    end