function [att0, res] = aligni0fit4(imu, pos, ts)
% SINS initial align based on inertial frame method & using polynomial fit.
%
% Prototype: [att0, res] = aligni0fit(imu, pos, ts)
% Inputs: imu - IMU data
%         pos - position
%         ts - IMU sampling interval
% Output: att0 - attitude align result
%         res - some other paramters for debug
%
% Example:
%     ts = 0.1;
%     imu = imustatic([0;0;10]*glv.deg, ts, 300);  imu([1:10,end-10:end],4:5)=1*ts; % acc disturb at start&end
%     [att0, res] = aligni0fit(imu, glv.pos0, ts);
%     
% See also  aligni0, alignfn, alignvn, aligncmps, alignWahba, alignsb.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/12/2012, 07/03/2014, 28/08/2014
global glv
    if nargin<3,  ts = imu(2,7)-imu(1,7);  end
    nn = 1; nts = nn*ts;  ratio = 1; % 0.995;
    len = fix(length(imu)/nn)*nn;
    eth = earth(pos);  lat = pos(1);  g0 = -eth.gn(3);
    qib0b = [1; 0; 0; 0];
    [vib0, vi0, pib0, pi0, vib0_1, vi0_1] = setvals(zeros(3,1));
    [pib0k, pi0k, vi0k, vib0k, fi0k, fib0k, attk, attkv] = prealloc(len/nn, 3);  tk = vib0k(:,1);
    k0 = fix(5/ts); % exculde the first 5s
    ki = timebar(nn, len, 'Initial align based on inertial frame & polyfit.');
    vel=1; odr=4; kfx.xk = zeros(odr,1); kfx.Pxk = eye(odr)*1e8; kfx.Hk = ones(1,odr);
    kfy = kfx; kfz = kfx;
    c = eye(3);
    for k=1:nn:len-nn+1
        wvm = imu(k:k+nn-1, 1:6);  kts = (k+nn-1)*ts;  tk(ki) = kts;
        [phim, dvbm] = cnscl(wvm);
        fib0 = qmulv(qib0b, dvbm)/nts;   % f
        vib0 = vib0 + fib0*nts;          % vel
        pib0 = ratio*pib0 + (vib0_1+vib0)*nts/2;  vib0_1 = vib0; % pos
%         fi0 = [eth.cl*cos(kts*glv.wie);eth.cl*sin(kts*glv.wie);eth.sl]*g0;
%         vi0 = vi0 + fi0*nts;
%         pi0 = ratio*pi0 + (vi0_1+vi0)*nts/2;      vi0_1 = vi0;
        [fi0, vi0, pi0] = i0fvp(kts, lat);
        qib0b = qupdt(qib0b, phim);  % qib0b updating
        pib0k(ki,:) = pib0'; vib0k(ki,:) = vib0'; fib0k(ki,:) = fib0'; % recording
        pi0k(ki,:) = pi0';   vi0k(ki,:) = vi0';   fi0k(ki,:) = fi0';
        kfx.Hk = kts.^(0:odr-1); kfy.Hk = kfx.Hk; kfz.Hk = kfx.Hk;
        if vel==0,     kfx = RLS(kfx, fib0(1));  kfy = RLS(kfy, fib0(2));  kfz = RLS(kfz, fib0(3));  % not good
        elseif vel==1, kfx = RLS(kfx, vib0(1));  kfy = RLS(kfy, vib0(2));  kfz = RLS(kfz, vib0(3));
        elseif vel==2, kfx = RLS(kfx, pib0(1));  kfy = RLS(kfy, pib0(2));  kfz = RLS(kfz, pib0(3));  end
        if k>k0
            k1 = fix(ki/2);
            swiet = sin(kts*glv.wie); cwiet = cos(kts*glv.wie);
            Cni0 = [-swiet,cwiet,0; 
                -eth.sl*cwiet,-eth.sl*swiet,eth.cl; 
                eth.cl*cwiet,eth.cl*swiet,eth.sl];
            qni0 = m2qua(Cni0);
            pcoef = flipud([kfx.xk,kfy.xk,kfz.xk])'; if vel>0, pcoef(:,end)=0; end
            [fi00, vi00, pi00] = i0fvp(tk(k1), lat);
            if vel==0,     qi0ib0 = dv2atti(fi00, fi0, polyvaln(pcoef,tk(k1)), polyvaln(pcoef,tk(ki)));
            elseif vel==1, qi0ib0 = dv2atti(vi00, vi0, polyvaln(pcoef,tk(k1)), polyvaln(pcoef,tk(ki)));  % 16/01/2023
            elseif vel==2, qi0ib0 = dv2atti(pi00, pi0, polyvaln(pcoef,tk(k1)), polyvaln(pcoef,tk(ki)));  end
            qnb = qmul(qmul(qni0,qi0ib0),qib0b);
            attkv(ki,:) = q2att(qnb)';    % using vib0/pib0 fit
            
            qi0ib0 = dv2atti(pi0k(k1,:)', pi0, pib0k(k1,:)', pib0);
            qnb = qmul(qmul(qni0,qi0ib0),qib0b);
            attk(ki,:) = q2att(qnb)';     % using pos
       end
       ki = timebar;
    end
%     figure, plot(tk, [vib0k(:,1:3)- [polyval(flipud(kfx.xk),tk), polyval(flipud(kfy.xk),tk), polyval(flipud(kfz.xk),tk)]]); xygo('\deltav fit err / m/s');
    k0 = fix(k0/nn)+1;
%     attk(1:k0,:) = repmat(attk(k0+1,:),k0,1);
    Cni0 = [0,1,0; -eth.sl,0,eth.cl;  eth.cl,0,eth.sl];
    att0 = q2att(qmul(m2qua(Cni0),qi0ib0));
    attk(1:k0,:) = repmat(att0',k0,1);
    attkv(1:k0,:) = repmat(attkv(k0+1,:),k0,1);
    attk(:,4) = tk; attkv(:,4) = tk;
    res = varpack(lat, nts, vib0k, pib0k, fib0k, vi0k, pi0k, fi0k, attk, attkv, att0); 
    att0 = attk(end,1:3)';
    resdisp('Initial align attitudes (arcdeg)', att0/glv.deg);
    ai0plot(nts, attk, attkv);
    
function ai0plot(ts, attk, attkv)
global glv
    t = (1:length(attk))'*ts;
    myfigure;
    subplot(211), plot(t, attk(:,1:2)/glv.deg), xygo('pr');
        hold on,  plot(t, attkv(:,1:2)/glv.deg, 'm:'),
    subplot(212), plot(t, attk(:,3)/glv.deg), xygo('y');
        hold on,  plot(t, attkv(:,3)/glv.deg, 'm:'), legend('i0 pos', 'i0fit vel');
