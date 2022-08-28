function [wn, s] = htwn(wp, s, len)
% Generate Heavy-tailed white noise.
%
% Prototype: wn = htwn(wp, enlarge, len)
% Inputs: wp - with probability
%         s - std enlarge with probability 'wp' 
%         len - element number 
% Outputs: wn - noise output
%          s - enlarge number for each wn
%
% Example1:
%    figure, plot(htwn(0.01, 100, 1000)); grid on
%
% Example2:
%    x=-30:0.1:30; n=1000; figure, plot(x,histc([randn(10000,1),htwn(0.3,10,10000)],x)/n); grid on
%
% See also  rstd.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/03/2022
    if length(wp)>1  % [wn, s] = htwn([wp1;wp2;...], [s1;s2;...], [len1;len2;...]);
        s1 = s;
        wn = []; s = [];
        for k=1:length(wp)
            [wnk,sk] = htwn(wp(k), s1(k), len(k));
            wn = [wn; wnk];  s = [s; sk];
        end
        return;
    end
    if nargin<3, len=1; end
    if nargin<2, s=100; end
    if nargin<1, wp=0.01; end
    if wp>1, len=wp; wp=0.01; end  % wn = htwn(len)
    wn = randn(len,1);
    r01 = rand(len,1);
    idx = r01 < wp;
    wn(idx) = wn(idx)*s;
    s(idx,1) = s;  s(~idx,1) = 1;
    