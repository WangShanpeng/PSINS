function [att, phi] = attpure(imu, avp, t0, t1, att0)
% Process pure attitude update.
%
% Prototype: [att, phi] = attpure(imu, avp, t0, t1, att0)
% Inputs: imu - SIMU data array
%         avp - avp ref, or initial avp0=[att0,vn0,pos0]
%         t0,t1 - start/end time.
%         att0 - initial attitude, if it exist, then avp=vp=gps.
%         
% Outputs: att - attitude update result
%          phi - attitude error
%
% See also  inspure, insinstant, drpure, attrvs.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/06/2021
    figcmp = 1;
    if nargin==5 && size(avp,2)<9, avp=[repmat(att0',length(avp),1),avp]; figcmp=0; end  % attpure(imu, gps, t0, t1, att0);
    if length(avp)==6, avp=[avp(1:3);0;0;0;avp(4:6)]; figcmp=0; end  % attpure(imu, [att0;pos0], t0, t1);
    if size(avp,2)==4, avp=[avp(:,1:3),zeros(length(avp),6),avp(:,4)]; figcmp=1; end  % attpure(imu, att, t0, t1);
    if size(avp,2)==1,  avp=repmat(avp', length(imu), 1); avp(:,end+1)=imu(:,end); figcmp=0; end
    if nargin<4, t1=min(imu(end,end),avp(end,end)); end
    if t1==inf, t1=min(imu(end,end),avp(end,end)); end
    if nargin<3, t0=max(imu(1,end),avp(1,end)); end
    if t0==-inf, t0=max(imu(1,end),avp(1,end)); end
    imu = datacut(imu, t0, t1);
    avp = interp1n(avp(:,[1:9,end]), imu(:,end));
    [nn, ts, nts] = nnts(2, imu(2,end)-imu(1,end));
    avp0 = avp(1,:)';
    if nargin==5, avp0(1:3)=att0; end
    ins = insinit(avp0, ts);
    len = length(imu);  lavp=length(avp);  att = zeros(fix(len/nn), 4);
    ki = timebar(nn, len, 'Pure attitude update processing.');
    for k=1:nn:len-nn+1
        k1 = k+nn-1;
        wvm = imu(k:k1, 1:6);  t = imu(k1,end);
        ins = insupdate(ins, wvm); 
        if k1<lavp,
            ins.vn = avp(k1,4:6)';  ins.pos = avp(k1,7:9)';
        end
        att(ki,:) = [ins.att; t]';
        ki = timebar;
    end
    if figcmp==1
        phi = avpcmpplot(avp(:,[1:3,end]), att, 'a', 'phi');
    else
        insplot(att,'a');
        phi = 0;
    end
