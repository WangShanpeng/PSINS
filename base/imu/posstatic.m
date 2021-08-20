function  pos = posstatic(pos, varargin)
% Make pos between t00-t01 no moving i.e. being static.
%
% Prototype: [imu, avp0, avp] = ap2imu(ap, ts)
% Inputs: pos - original position
%         varargin - = t00,t01, t10,t11, ...
% Output: pos - position after static setting
%
% See also  apmove, tsetflag.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/01/2021
    dpos = diff([pos(1,1:3); pos(:,1:3)]);
    t = pos(:,end);
    for k=1:length(varargin)/2
        t00 = varargin{2*k-1}; t01 = varargin{2*k};
        dpos((t00<=t & t<t01),:) = 0;
    end
    pos(:,1:3) = cumsum(dpos,1);