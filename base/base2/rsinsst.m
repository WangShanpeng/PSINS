function [avp, ierr] = rsinsst(imu, avp0, T, isfig)
% Static base Rotation-SINS pure inertial navigation
% using initial condition avp0 = [att0,vn0,pos0].
%
% Prototype: avp = inspure(imu, avp0, href, isfig)
% Inputs: imu - SIMU data array
%         avp0 - initial parameters, avp0 = [att0,vn0,pos0]
%         T - navigation time
%         isfig - figure flag
% Output: avp - navigation results, avp = [att,vn,pos,t]
%
% See also  inspure, inspurest.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/10/2024
global glv
    [nn, ts, nts] = nnts(2, imu(2,end)-imu(1,end));
    ierr = diff(imu([1,end],1:6))/ts;  ierr = [ierr(1:3)/glv.dph; ierr(4:6)/glv.ug],
    L = length(imu)-nn+1;
    len = fix(T/ts);    avp = zeros(fix(len/nn), 10);
    ins = insinit(avp0, ts);
    ki = timebar(nn, len, 'RSINS pure inertial navigation processing.');
    k0 = 1;
    for k=1:nn:len-nn+1
        if k0>L, k0=1; end
        k1 = k0+nn-1;
        wvm = imu(k0:k1, 1:6);  t = (k+nn-1)*ts;
        ins = insupdate(ins, wvm);  ins.pos(3)=avp0(9); ins.vn(3)=avp0(6);
        avp(ki,:) = [ins.avp; t]';
        ki = timebar;  k0 = k0+nn;
    end
    if nargin<4, isfig=1; end
    if isfig==1
        tscalepush('t/h');
        insplot(avp);
        tscalepop();
    end

