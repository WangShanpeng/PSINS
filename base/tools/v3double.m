function v1v3 = v3double(v3v1)
% Translate 3-column std vector within [-45000,45000] to 1-column double vector,
% or vice versa.
%
% Prototype: v1v3 = v3double(v3v1)
% Input: v3v1 - 3-column std vector within [-45000,45000], or 1-column vector
% Output: v1v3 - 1-column double vector, or 3-column std vector within [-45000,45000]
% Example:
%    v3=[10000;-2000;-3000000]; v1=v3double(v3); [v3,v3double(v1)]
%    v3=round(randn(10,3)*1e4*2); v1=v3double(v3); myfig(v3); plot(v3double(v1),'-.')
%
% See also  v2double, v5double.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/05/2022
    [m,n] = size(v3v1);
    if m<=3&&n==1, v3v1=v3v1'; [m,n] = size(v3v1); end
    if n>1  % v3->v1
        if n<3, v3v1 = [v3v1, ones(m,3-n)]; end
        v3v1(v3v1>45000) = 45000;
        idx=v3v1<-45000; v3v1(idx) = -45000; v3v1(idx) = v3v1(idx)+100000;
        v1v3 = round(v3v1(:,1)) + round(v3v1(:,2))/1e5 + round(v3v1(:,3))/1e10;
    else % v1->v3
        v1v3(:,1) = floor(v3v1+eps);  v3v1=(v3v1-v1v3(:,1))*1e5;
        v1v3(:,2) = floor(v3v1+eps);  v3v1=(v3v1-v1v3(:,2))*1e5;
        v1v3(:,3) = floor(v3v1+eps);
        idx=v1v3>50000; v1v3(idx) = v1v3(idx)-100000;
        if size(v1v3,1)==1, v1v3=v1v3'; end
    end