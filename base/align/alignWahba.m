function [att0, attk] = alignWahba(imu, pos, isfig)
% SINS initial align based on inertial frame and Wahba method. In the Wahba
% problem, it is solved by SVD & QUEST method (equivalent).
%
% Prototype: [att0, attk] = alignWahba(imu, pos, isfig)
% Inputs: imu - IMU data
%         pos - position
%         isfig - figure flag
% Output: att0 - attitude align result
%
% Example1:
%     glvs;
%     [imu, avp0, ts] = imufile('lasergyro.imu');
%     att = alignWahba(imu(1:300/ts,:), avp0(7:9)');
%
% Example2:
%     glvs;
%     avp0 = [randn(3,1)*0;zeros(3,1);glv.pos0*0];
%     [imu, eth] = imustatic(avp0,1,3000,imuerrset(0,0,1.1,100));
%     att = alignWahba(imu, avp0(7:9)');
% 
% See also  alignfn, alignvn, aligncmps, aligni0, alignsb.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/06/2012, 20/01/2021
global glv
    if nargin<3,  isfig = 1; end
	[nn, ts, nts] = nnts(2, diff(imu(1:2,end)));
    len = fix(length(imu)/nn)*nn;
	eth = earth(pos);   g0 = -eth.gn(3);
    qib0b = [1; 0; 0; 0];
    [vib0, vi0] = setvals(zeros(3,1));
    K = zeros(4); A = zeros(3); Af = zeros(3);
    attk = zeros(len/nn, 4); attk_SVD = attk; testA = zeros(len/nn, 7);
    ki = timebar(nn, len, 'Initial align using i0 & Wahba method.');
    for k=1:nn:len-nn+1
        wvm = imu(k:k+nn-1, 1:6);  kts = (k+nn-1)*ts;   timu = imu(k+nn-1,end);
        [phim, dvbm] = cnscl(wvm);
        fib0 = qmulv(qib0b, dvbm); vib0 = 1*vib0 + fib0;
        fi0 = [eth.cl*cos(kts*glv.wie); eth.cl*sin(kts*glv.wie); eth.sl]*g0*nts; vi0 = 1*vi0 + fi0;
        qib0b = qupdt(qib0b, phim);
        dM = rq2m([0;vib0])-lq2m([0;vi0]);  % QUEST method
        K = 1.0*K + dM'*dM*nts;
        [v, d] = eig(K);  qi0ib0 = v(:,1); 
        Cni0 = pos2cen([pos(1); kts*glv.wie; 0])'; qni0 = m2qua(Cni0);
        qnb = qmul(qmul(qni0,qi0ib0),qib0b);
        attk(ki,:) = [q2att(qnb)', timu];
        A = A + vi0*vib0';  % SVD method
%         [U, S, V]=svd(A);  qi0ib0_SVD = m2qua(U*V');  % not good
%         C = svdest(A);  qi0ib0_SVD = m2qua(C);
        C = foam(A);  qi0ib0_SVD = m2qua(C);  % FOAM method
        if k>10
            testA(ki,:) = [vi0', vib0', kts];
            % testA(ki,:) = [cond(A),det(A),sign(det(A)), det(U)*det(V), kts];
            attk_SVD(ki,:) = [q2att(qmul(qmul(qni0,qi0ib0_SVD),qib0b))', kts];
        end
        ki = timebar;
    end
    attk(ki:end,:) = []; attk_SVD(ki:end,:) = [];
    att0 = attk(end,1:3)';
    t = attk(:,end);
    resdisp('Initial align attitudes (arcdeg)', att0/glv.deg);
    if isfig
        insplot(attk);
        subplot(211), plot(t, attk_SVD(:,1:2)/glv.deg, 'm');
        subplot(212), plot(t, attk_SVD(:,3)/glv.deg, 'm');
        legend('QUEST Wahba', 'SVD Wahba');
    %     mysemilogy(testA(1:ki,end), testA(1:ki,1:2));
    end