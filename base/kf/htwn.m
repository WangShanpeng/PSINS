function wn = htwn(wp, s, len)
% Generate Heavy-tailed white noise.
%
% Prototype: wn = htwn(wp, enlarge, len)
% Inputs: wp - with probability
%         s - std enlarge with probability 'wp' 
%         len - element number 
% Output: wn - noise output
%
% Example:
%    figure, plot(htwn(0.01, 100, 1000)); grid on
%
% See also  N/A.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/03/2022
    if nargin<3, len=1; end
    if nargin<2, s=100; end
    if nargin<1, wp=0.01; end
    if wp>=1, len=wp; wp=0.01; end  % wn = htwn(len)
    wn = randn(len,1);
    r01 = rand(len,1);
    idx = r01 < wp;
    wn(idx) = wn(idx)*s;
    