function avp = avplever(avp, lever, tDelay)
% AVP lever arm monitoring or compensation.
%
% Prototype: avp = avplever(avp, lever)
% Inputs: avp - initial AVP parameter array
%         lever - lever arms [lx^b; ly^b; lz^b];
%         tDelay - time delay;
% Output: avp - AVP parameter array after lever arm compensation
%
% Example:
%    avp1 = avplever(avp, xkpk(end,16:18)', xkpk(end,19));
%
% See also  pp2lever, inslever.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/01/2021
    ts = diff(avp(1:2,end));
    eth = earth(avp(1,7:9)');
    Mpv = [0, 1/eth.RMh, 0; 1/eth.clRNh, 0, 0; 0, 0, 1];
    Cnb_1 = a2mat(avp(1,1:3)');
    for k=2:size(avp,1)
        Cnb = a2mat(avp(k,1:3)');
        wib = m2rv(Cnb_1'*Cnb)/ts;  % ~=web
        CW = Cnb*askew(wib);
        MpvCnb = Mpv*Cnb;
        avp(k,4:9) = avp(k,4:9)+ [CW*lever; MpvCnb*lever]';
        Cnb_1 = Cnb;
    end
    avp(1,4:9) = avp(2,4:9);
    if nargin==3
        avp = avpinterp1(avp, avp(:,end)+tDelay);
        avp(:,end) = avp(:,end)-tDelay;
    end

