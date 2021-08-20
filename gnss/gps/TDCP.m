function baseline = TDCP(satPos1, satPos2, recPos1, recPos2, dphi, ths)
% Time-Differenced Carrier Phase algorithm.
%
% Prototype: baseline = TDCP(satPos1, satPos2, recPos1, recPos2, dphi, ths)
% Inputs: satPos1 - satellite positions at time 1
%         satPos2 - satellite positions at time 2
%         recPos1 - receiver positions at time 1
%         recPos2 - receiver positions at time 2
%         dphi - time-differenced carrier phase between time 1 & time 2
%         ths - iteration threshold
% Output: baseline - [dX, dY, dZ, dt] position and time differences
%                between time 1 & time 2.
%
% See also  spp, lsPos, satPosVel.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/07/2015
    if nargin<6,  ths = 0.001;  end
    baseline = recPos2-recPos1;
    satNum = length(dphi);
    S1R1 = zeros(satNum,1);  S2R1 = zeros(3,satNum);
    for k=1:satNum
        S1R1(k) = norm(satPos1(:,k)-recPos1(1:3));
        S2R1(:,k) = satPos2(:,k) - recPos1(1:3);
    end
    SS = zeros(satNum,1); A = zeros(satNum,4);
    for kk=1:20
        for k=1:satNum
            S2R1k = S2R1(:,k);
            n = norm(S2R1k-baseline(1:3));
            SS(k,1) = S2R1k'*S2R1k/n;
            A(k,:) = [-(2*S2R1k-baseline(1:3))'/n, 1];
        end
        x = (A'*A)^-1*A'*(dphi-SS+S1R1);
        if norm(baseline-x)<ths;  break;  end
        baseline = x;
    end
%     kk
	baseline = x;

