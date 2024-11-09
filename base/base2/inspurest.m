function avp = inspurest(ap0, ts, T, imuerr, davp, href)
% Process SINS pure inertial navigation on static base.
%
% Prototype: avp = inspurest(ap0, ts, T, imuerr, davp, href)
% Inputs: ap0 - initial parameters, ap0 = [att0,pos0]
%         ts - sampling interval, NOTE: ts<0 for self initial alignment !!!
%         T - simulation time length
%         imuerr - imu error struct
%         davp - avp error
%         href -    'v' - velocity fix-damping, =vn0
%                   'V' - vertical velocity fix-damping, =vn0(3)
%                   'p' - position fix-damping, =pos0
%                   'P' - position fix-damping, =pos0 & vertical velocity fix-damping, =vn0(3)
%                   'H' - height fix-damping, =pos0(3)
%                   'f' - height free
% Output: avp - navigation results, avp = [att,vn,pos,t]
%
% Example:
%     avp = inspurest([], 1, 3600*24, imuerrset(0.01,10,0.0000,1));
%
% See also  inspure, rsinsst.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/10/2024
global glv
    if nargin<6, href='V'; end
    if nargin<5, davp=zeros(9,1); end
    if nargin<4, imuerr=imuerrset(0,0); end
    if nargin<3, T=3600; end
    if nargin<2, ts=0.1; end
    if nargin<1, ap0=[0;0;0; glv.pos0]; end
    if     length(ap0)<3, ap0=[0;0;0; 0;0;0; glv.pos0];
    elseif length(ap0)<6, ap0=[ap0(1:3); 0;0;0; glv.pos0];
    elseif length(ap0)<9, ap0=[ap0(1:3); 0;0;0; ap0(4:6)]; end
    self_align = 0;
    if ts<0, self_align=1; ts=-ts; end
    imu0 = imustatic(avpadderr(ap0,davp), ts, 3600*ts);
    imu = imuadderr(imu0, imuerr);
    if self_align==1,  ap0(1:3) = alignsb(imu, ap0(7:9)); end
    [nn, ts, nts] = nnts(2, ts);
    L = length(imu0)-nn+1;  k0 = 1;
    len = fix(T/ts);    avp = zeros(fix(len/nn), 10);
    ins = insinit(ap0, ts);  vn1=ins.vn; pos1=ins.pos;
    ki = timebar(nn, len, 'Static pure inertial navigation processing.');
    for k=1:nn:len-nn+1
        if k0>L,  k0=1; imu = imuadderr(imu0, imuerr);  end
        k1 = k0+nn-1;
        wvm = imu(k0:k1, 1:6);  t = (k+nn-1)*ts;
        ins = insupdate(ins, wvm);
        if href=='v',      ins.vn = vn1;
        elseif href=='V',  ins.vn(3) = vn1(3);
        elseif href=='p',  ins.pos = pos1;
        elseif href=='P',  ins.pos = pos1;  ins.vn(3) = vn1(3);
        elseif href=='H',  ins.pos(3) = pos1(3);
        elseif href=='f',  
            if ins.pos(3)>glv.Re*1.01, ins.pos(3)=glv.Re*1.01;  % 67km
            elseif ins.pos(3)<glv.Re*0.99, ins.pos(3)=glv.Re*0.99;  end
        else
            error('No SINS-pure type matched!');
        end
        avp(ki,:) = [ins.avp; t]';
        ki = timebar;
    end
    insplot(avp);
