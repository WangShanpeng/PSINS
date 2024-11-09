function avp = inspurervs(imu, avp1, href, isfig)
% Process reverse SINS pure inertial navigation with SIMU log data and
% using avp1 at imu(:,end+1).
%
% Prototype: avp = inspurervs(imu, avp1, href, isfig)
% Inputs: imu - SIMU data array
%         avp0 - initial parameters, avp0 = [att0,vn0,pos0]
%         href -    'v' - velocity fix-damping, =vn0
%                   'V' - vertical velocity fix-damping, =vn0(3)
%                   'p' - position fix-damping, =pos0
%                   'P' - position fix-damping, =pos0 & vertical velocity fix-damping, =vn0(3)
%                   'H' - height fix-damping, =pos0(3)
%                   'f' - height free.
%                   'O' - open loop, vn=0.  Ref: my PhD thesis P41
%         isfig - figure on/off flag
% Output: avp - navigation results, avp = [att,vn,pos,t]
%
% Example:
%     t0=0; t1=600; t2=t1+300;
%     att1 = aligni0(datacut(imu,t1,t2), getat(gps(:,4:end),t1));
%     avp = inspurervs(datacut(imu,t0,t1), [att1;getat(gps(:,4:end),t1)], 'f');
%
% See also  inspure, attpure, attrvs, avpinv, POSProcessing.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/09/2024
    [nn, ts, nts] = nnts(2, imu(2,end)-imu(1,end));
    if abs(norm(avp1(1:4))-1)<1e-6, avp1(1:3)=q2att(avp1(1:4)); avp1(4)=[]; end % avp0=[qnb; ...]
    if length(avp1)<9, avp1=[avp1(1:3);zeros(3,1);avp1(4:end)]; end  % avp0=[att;pos]
    ins = insinit(avp1, ts);  vn1 = avp1(4:6); pos1 = avp1(7:9);
    ins.eth.wie = -ins.eth.wie;  ins.vn = -ins.vn; ins.eb = ins.eb;
    vp_fix = href;
    len = length(imu);    avp = zeros(fix(len/nn), 10);
    ki = timebar(nn, len, 'Reverse pure inertial navigation processing.');
    for k=len:-nn:nn+1
        ik1 = k-nn+1; wvm = imu(k:-1:ik1,1:6); wvm(:,1:3) = -wvm(:,1:3); t = imu(ik1-1,7);
        ins = insupdate(ins, wvm);
        if vp_fix=='v',      ins.vn = vn1;
        elseif vp_fix=='V',  ins.vn(3) = vn1(3);
        elseif vp_fix=='p',  ins.pos = pos1;
        elseif vp_fix=='P',  ins.pos = pos1;  ins.vn(3) = vn1(3);
        elseif vp_fix=='H',  ins.pos(3) = pos1(3);
        elseif vp_fix=='f',
            ins.vn(3) = ins.vn(3);  % free, no need
        elseif vp_fix=='O',
            ins.vn0 = zeros(3,1);  % duplicate, no need init again
            ins.openloop = 1;
        else
            error('No SINS-pure type matched!');
        end
        avp(ki,:) = [ins.avp; t]';
        ki = timebar;
    end
    avp = flipud(avp(1:ki-1,:));  avp(1,7:9)=avp(end,7:9);
    if nargin<4, isfig=1; end
    if isfig==1,
        insplot(avp);
    end
