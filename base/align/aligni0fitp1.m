function [att0, res] = aligni0fitp(imu, pos, isfig)
% SINS initial align based on inertial frame method & using position curve fit.
%
% Prototype: [att0, res] = aligni0fit(imu, pos, isfig)
% Inputs: imu - IMU data
%         pos - position
%         isfig - figure flag
% Output: att0 - attitude align result
%         res - some other paramters for debug
%
% Example:
%     glvs; ts = 0.1;  pos0=[-83*glv.deg;0;0];
%     imu = imustatic([[0;0;10]*glv.deg; pos0], ts, 600);  imu([1:10,end-9:end],4:5)=10*ts; % acc disturb at start&end
%     [att0, res] = aligni0fitp1(imu, pos0);
%     
% See also  aligni0fitv, aligni0, alignfn, alignvn, aligncmps, alignWahba, alignsb.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2023
global glv
    if nargin<3,  isfig = 1; end
    [nn, ts, nts] = nnts(1, diff(imu(1:2,end)));
    ratio = 1.0;
    len = fix(length(imu)/nn)*nn;
    eth = earth(pos);  lat = pos(1);
    qib0b = [1; 0; 0; 0];  qni00 = m2qua(pos2cen(pos(1))'); 
    [vib0, pib0, vib0_1] = setvals(zeros(3,1));
    [pib0k, pi0k, vi0k, vib0k, fi0k, fib0k, attk, attkv, vn0, vnk, posk] = prealloc(len/nn, 3);  tk = vib0k(:,1);  timu = tk;
    k0 = fix(5/ts); % exculde the first 5s
    ki = timebar(nn, len, 'Initial align based on inertial frame & curve-fit.');
    G = glv.g0*eth.cl/glv.wie^2;
    xk = zeros(5,3); Pxk = diag([G*[1;1;1];10;10]*100)^2;  [UU, DD] = UDUT(Pxk);                xk1=xk; Pxk1=Pxk; 
    for k=1:nn:len-nn+1
        wvm = imu(k:k+nn-1, 1:6);  kts = (k+nn-1)*ts;  tk(ki) = kts;  timu(ki) = imu(k+nn-1,end);
        [phim, dvbm] = cnscl(wvm);
        fib0 = qmulv(qib0b, dvbm)/nts;   % f
        vib0 = vib0 + fib0*nts;          % vel
        pib0 = ratio*pib0 + (vib0_1+vib0)*nts/2;  vib0_1 = vib0; % pos
        [fi0, vi0, pi0] = i0fvp(kts, lat);
        qib0b = qupdt(qib0b, phim);  % qib0b updating
        pib0k(ki,:) = pib0'; vib0k(ki,:) = vib0'; fib0k(ki,:) = fib0'; % recording
        pi0k(ki,:)  = pi0';   vi0k(ki,:) = vi0';   fi0k(ki,:) = fi0';
        wt = kts*glv.wie;  Hk = [1-cos(wt)-wt*wt/2, wt-sin(wt), wt*wt/2, wt, 1];
        Rk = 1;  % if kts<10, Rk = 10^2; end
        Pxzk = Pxk*Hk';  Pzk = Hk*Pxzk + Rk;  Kk = Pxzk*Pzk^-1;  Pxk = Pxk - Kk*Pxzk';  Pxk = (Pxk+Pxk')*0.5;   % RLS
%         [UU, DD, Kk] = RLSUD(UU, DD, Hk, Rk);
        pib02 = (Hk*xk)'; rk = pib0-pib02; xk = xk + [Kk*rk(1),Kk*rk(2),Kk*rk(3)];
        
        wt = kts*glv.wie;  Hk1 = [1-cos(wt), wt-sin(wt), wt*wt/2, wt, 1];
        Rk1 = 1;  % if kts<10, Rk = 10^2; end
        Pxzk1 = Pxk1*Hk1';  Pzk1 = Hk1*Pxzk1 + Rk1;  Kk1 = Pxzk1*Pzk1^-1;  Pxk1 = Pxk1 - Kk1*Pxzk1';  Pxk1 = (Pxk1+Pxk1')*0.5;   % RLS
        pib021 = (Hk1*xk1)'; rk1 = pib0-pib021; xk1 = xk1 + [Kk1*rk1(1),Kk1*rk1(2),Kk1*rk1(3)];        
        
        if k>k0
            k1 = fix(ki/2);
            swiet = sin(kts*glv.wie); cwiet = cos(kts*glv.wie);
            Cni0 = [-swiet,cwiet,0; 
                -eth.sl*cwiet,-eth.sl*swiet,eth.cl; 
                eth.cl*cwiet,eth.cl*swiet,eth.sl];
            qni0 = m2qua(Cni0);
            wt = tk(k1)*glv.wie; pib01 = ([1-cos(wt)-wt*wt/2, wt-sin(wt), wt*wt/2, 0, 0]*xk)';
            wt = kts*glv.wie; pib02 = ([1-cos(wt)-wt*wt/2, wt-sin(wt), wt*wt/2, 0, 0]*xk)';
            qi0ib0 = dv2atti(pi0k(k1,:)', pi0, pib01, pib02);  % 30/01/2023
            qnib0 = qmul(qni0,qi0ib0);
            qnb = qmul(qnib0,qib0b);
            attkv(ki,:) = q2att(qnb)';    % using pib0 fit
            vn0(ki,:) = -qmulv(qmul(qni00,qi0ib0),glv.wie*xk(end-1,:)');
            vnk(ki,:) = qmulv(qnib0,vib0-(glv.wie*[sin(wt)-wt,1-cos(wt),wt,1,0]*xk)');
            posk(ki,:) = qmulv(qnib0,rk)'; % posk(ki,:) = qmulv(qnib0,xk(end,:)')';
            qi0ib0 = dv2atti(pi0k(k1,:)', pi0, pib0k(k1,:)', pib0);
            qnb = qmul(qmul(qni0,qi0ib0),qib0b);
            attk(ki,:) = q2att(qnb)';     % using pos
       end
       ki = timebar;
    end
    lti = [acos([norm(xk(1,:)),norm(xk(2,:))]*glv.wie^2/glv.g0), asin(norm(xk(3,:)-xk(1,:))*glv.wie^2/glv.g0)]/glv.deg  % lat estimate
    sign(cross(xk(1,:),xk(2,:))*xk(3,:)')*atan(norm(xk(3,:)-xk(1,:))/norm(xk(2,:)))/glv.deg
    lti1 = [acos([norm(xk1(1,:)),norm(xk1(2,:))]*glv.wie^2/glv.g0), asin(norm(xk1(3,:))*glv.wie^2/glv.g0)]/glv.deg  % lat estimate
    asin(xk1(3,:)/norm(xk1(2,:)))/glv.deg
    k0 = fix(k0/nn)+1;
%     attk(1:k0,:) = repmat(attk(k0+1,:),k0,1);
    Cni0 = [0,1,0; -eth.sl,0,eth.cl;  eth.cl,0,eth.sl];
    att0 = q2att(qmul(m2qua(Cni0),qi0ib0));
    attk(1:k0,:) = repmat(att0',k0,1);
    attkv(1:k0,:) = repmat(attkv(k0+1,:),k0,1);
    attk(:,4) = timu; attkv(:,4) = timu;
    res = varpack(lat, nts, vib0k, pib0k, fib0k, vi0k, pi0k, fi0k, attk, attkv, att0, vn0, vnk, posk); 
    att0 = attkv(end,1:3)';
    resdisp('Initial align attitudes (arcdeg)', att0/glv.deg);
    if isfig, ai0plot(timu, attk, attkv, vn0, vnk, posk); end
    
function ai0plot(t, attk, attkv, vn0, vnk, posk)
global glv
    myfigure;
    subplot(221), plot(t, attk(:,1:2)/glv.deg), xygo('pr');
        hold on,  plot(t, attkv(:,1:2)/glv.deg, 'm:'),
    subplot(223), plot(t, attk(:,3)/glv.deg), xygo('y');
        hold on,  plot(t, attkv(:,3)/glv.deg, 'm:'), legend('i0 pos', 'i0fit pos');
    subplot(322), plot(t, vn0), xygo('vn0 / m/s');
    subplot(324), plot(t, vnk), xygo('vnt / m/s');
    subplot(326), plot(t, posk), xygo('post / m');

