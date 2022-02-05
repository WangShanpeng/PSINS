function [qis, utc0] = cnssimu(avp, ns, mubs, utc0)
% CNS attitude quaternion 'qis' simulator
%
% Prototype: [qis, utc0] = cnssimu(avp, datt, mubs, utc0)
% Inputs: avp - avp info, always from trajectory simulation
%         ns - CNS attitude observation noise
%         mubs - install angles to form DCM 'Cbs'
%         utc0 - = [year,month,day,second,dUT1,dTAI]
% Outputs: qis - = [q1, q2, q3, t] array
%         utc0 - UTC info
%          
% Example:
%   [qis, utc0] = cnssimu(avp, [10;10;30]*glv.sec, [10;10;30]*glv.min, [2021;11;22;12*3600; -0.1;37]);
%
% See also  gpssimu, trjsimu, odsimu, bhsimu.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/11/2021
global glv
    if nargin<4,  utc0=[2021;11;22;0.0;-0.1;37];   end
    if nargin<3,  mubs=[0;0;0];   end
    if nargin<2,  ns=[0;0;0];   end
    Cie = cnsCie(utc0(1:3), utc0(4), utc0(5), utc0(6));
    Cbs = a2mat(mubs);
    qis = avp(:,[1:3,end]);
    len = length(avp);
    timebar(1, len, 'CNS quaternion(qis) Simulation.');
    for k=1:len
        Cniose = a2mat(ns.*randn(3,1));
        qq = m2qua(Cie*rxyz(glv.wie*avp(k,end))*pos2cen(avp(k,7:9)')*a2mat(avp(k,1:3)')*Cbs*Cniose);
        if qq(1)>=0, qis(k,1:3) = qq(2:4)'; else, qis(k,1:3) = -qq(2:4)'; end  % make sure q(1)>=0
        timebar;
    end
    myfig, plot(qis(:,end), qis(:,1:3)); xygo('q ^i_s (2,3,4)');
