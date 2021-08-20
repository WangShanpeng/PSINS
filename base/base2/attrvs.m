function [att0, attk] = attrvs(imu, att1, pos)
% Process pure attitude reverse update.
%
% Prototype: [att0, attk] = attrvs(imu, att1, pos)
% Inputs: imu - SIMU data array
%         att1 - attitude at end time 'imu(end,end)'
%         pos - position
%         
% Outputs: att0 - attitude at start time 'imu(1,end)'
%          attk - attitude serial record
%
% Example:
%     glvs
%     [imu, avp0, ts] = imufile('lasergyro');
%     att = attpure(imu, avp0, 1, 100);
%     [att0, iatt] = attrvs(datacut(imu,10,90), getat(att,90), avp0(7:9));
%     avpcmpplot(att, iatt, 'a', 'phi');
%
% See also  attpure.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/07/2021
    if length(att1)>7, pos=att1(7:9); att1=att1(1:3);        % [att0, attk] = iatt(imu, [att1;vn;pos])
    elseif length(att1)>4, pos=att1(4:6); att1=att1(1:3); end  % [att0, attk] = iatt(imu, [att1;pos])
    [nn, ts, nts] = nnts(4, diff(imu(1:2,end)));
    len = length(imu);
    eth = earth(pos);  wniets=eth.wnie*nts;
    qnb = a2qua(att1);
    attk = zeros(fix(len/nn),4); kk=1;
    timebar(nn, len, 'Pure attitude reverse update processing.');
    for k=len:-nn:nn+1
        [phim, dvbm] = cnscl(imu(k:-1:k-nn+1,:));
        qnb = qupdt2(qnb, -phim, -wniets);
        attk(kk,:) = [q2att(qnb);imu(k-nn,end)]';  kk=kk+1;
        timebar(nn);
    end
    attk(kk:end,:) = [];    attk = flipud(attk);
    att0 = q2att(qnb);