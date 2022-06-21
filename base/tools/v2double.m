function v1v2 = v2double(v2v1)
% Translate 2-column std vector within [-45000000,45000000] to 1-column double vector,
% or vice versa.
%
% Prototype: v1v2 = v2double(v2v1)
% Input: v2v1 - 2-column std vector within [-45000000,45000000], or 1-column vector
% Output: v1v2 - 1-column double vector, or 2-column std vector within [-45000000,45000000]
% Example:
%    v2=[100000;-10000]; v1=v2double(v2); [v2,v2double(v1)]
%    v2=round(randn(100,2)*1e2); v1=v2double(v2); myfig(v2); plot(v2double(v1),'-.')
%
% See also  v3double, v5double.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/05/2022
    [m,n] = size(v2v1);
    if m==2&&n==1, v2v1=v2v1'; [m,n] = size(v2v1); end
    if n==2  % v2->v1
        v2v1(v2v1>45000000) = 45000000;
        idx=v2v1<-45000000; v2v1(idx) = -45000000; v2v1(idx) = v2v1(idx)+100000000;
        v1v2 = round(v2v1(:,1)) + round(v2v1(:,2))/1e8;
    else % v1->v2
        v1v2(:,1) = floor(v2v1+eps);  v2v1=(v2v1-v1v2(:,1))*1e8;
        v1v2(:,2) = floor(v2v1+eps);
        idx=v1v2>50000000; v1v2(idx) = v1v2(idx)-100000000;
        if size(v1v2,1)==1, v1v2=v1v2'; end
    end