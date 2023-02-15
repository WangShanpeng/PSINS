function [avp, tn] = insinterm(imu, avpr, t0, t1, tint, ts)
% Process pure intermittent INS for short time within [t0,t1], for SW using.
%
% Prototype: [avp, tn] = insintermit(imu, avpr, t0, t1, tint)
% Inputs: imu - SIMU data array
%         avpr - AVP parameters, avp = [att,vn,pos,eb,db,t]
%         t0 - start time in second.
%         t1 - end time in second.
%         tint - time interval.
%         ts - IMU sampling interval.
% Outputs: avp - navigation results, avp = [att,vn,pos,t]
%          tn - intermittent INS start time sequence
%
% See also  insinstant, inspure.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/12/2022
    if nargin<6, ts=diff(imu(1:2,end)); end
    if nargin<5, tint=300; end
    if nargin<3, t0=imu(1,end); t1=imu(end,end); end  % [avp, tn] = insinterm(imu, avpr)
    if nargin==3, tint=t0; t0=imu(1,end); t1=imu(end,end); end  % [avp, tn] = insinterm(imu, avpr, tint)
    [nn, ts, nts] = nnts(2, ts);
    if size(avpr,2)<16, avpr=[avpr(:,1:9), zeros(length(avpr),6), avpr(:,end)]; end
    imuavp = combinedata(imu, avpr);
    k0=find(imuavp(:,7)>t0,1); k1=find(imuavp(:,7)>t1,1);
    if isempty(k0), k0=1; end
    if isempty(k1), k1=length(imuavp); end
    if imuavp(k0,14)==0, k0=k0+1; end  % maybe two-sample avp
    t00 = 0;
    ins = insinit(imuavp(k0,8:16)', ts);  ins.eb = imuavp(k0,17:19)'; ins.db = imuavp(k0,20:22)';
    avp = imuavp(k0:nn:k1,[8:16,end]);  kk=1;
    tn = avp(:,end); kn=1;
    timebar(nn, k1-k0);
    for k=k0:nn:k1-nn
        t00 = t00+nts;
        if t00>tint
            ins = insinit(imuavp(k,8:16)', ts); t00=0;   ins.eb = imuavp(k,17:19)'; ins.db = imuavp(k,20:22)';
            tn(kn) = imuavp(k,7); kn=kn+1;
        end
        wvm = imuavp(k:k+nn-1,1:6);
        ins = insupdate(ins, wvm);
        avp(kk,:) = [ins.avp; imuavp(k+nn-1,7)]'; kk=kk+1;
        timebar(nn);
    end
    tn(kn:end)=[];
