function imu = imuqn(imu, qng, qna)
% SIMU add quantization noise.
%
% Prototype: imu = imuqn(imu, qng, qna)
% Inputs: imu - raw SIMU data
%         qng - gyro quantization noise, like RLG, in arcsec or rad
%         qna- acc quantization noise, like I/F convertor, in pulse/g.s
% Output: imu - output SIMU data with quantization noise
%
% Example
%   glvs; avp0 = avpset([0.0001;0.02;1]*glv.deg, 0, glv.pos0, 0);
%   imu = imustatic(avp0, 0.1, 180);
%   imu1 = imuqn(imu); imuplot(imu1);  % inspure(imu1, avp0, 'f');
%   alignvn(imu1, avp0(1:3), avp0(7:9));
%
% See also  imuadderr, avarsimu.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/12/2022
global glv
    if nargin<3, qna=10000; end
    if nargin<2, qng=0.466; end  % RLG 0.466 arcsec
    if length(qng)==1; qng=repmat(qng,3,1); end
    if length(qna)==1; qna=repmat(qna,3,1); end
    for k=1:3  % gyro
        if qng(k)==0, continue; end
        if qng(k)>10*glv.sec, qng(k)=qng(k)*glv.sec; end  % if in arcsec
        imu(:,k) = diff([0;fix(cumsum(imu(:,k))/qng(k))])*qng(k);
    end
    for k=1:3  % acc
        if qna(k)==0, continue; end
        qna(k) = glv.g0*1/qna(k);
        imu(:,k+3) = diff([0;fix(cumsum(imu(:,k+3))/qna(k))])*qna(k);
    end
    