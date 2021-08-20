function [att0, res] = aligni0noL(imu, pos, ts)
% SINS initial align based on inertial frame method with no lat input.
%
% Prototype: [att0, res] = aligni0(imu, pos, ts)
% Inputs: imu - IMU data
%         pos - position
%         ts - IMU sampling interval
% Output: att0 - attitude align result
%         res - some other paramters for debug
%
% Example:
%     glvs;
%     [imu, avp0, ts] = imufile('lasergyro.imu');
%     [imu1, avp0] = imuoppl(imu, avp0);
%     att = aligni0noL(imu1(1:300/ts,:), -avp0(7:9)');
%
% See also  aligni0noL, alignfn, alignvn, aligni0vn, aligncmps, alignWahba, alignsb, i0fvp.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/01/2021
global glv
    if nargin<3,  ts = imu(2,end)-imu(1,end);  end
    nn = 2; nts = nn*ts;  ratio = 1; % 0.995;
    len = fix(length(imu)/nn)*nn;
    if length(pos)>4 , pos=pos(4:6); 
    elseif length(pos)==1, pos=[pos;0;0]; end
    eth = earth(pos);  lat = pos(1);  g0 = -eth.gn(3);
    qib0b = [1; 0; 0; 0];
    [vib0, vi0, pib0, pi0, vib0_1, vi0_1] = setvals(zeros(3,1));
    [pib0k, pi0k, vi0k, vib0k, fi0k, fib0k, attk, attkv] = prealloc(len/nn, 3);
    Lk = zeros(len/nn,4);
    k0 = fix(5/ts); % exculde the first 5s
    ki = timebar(nn, len, 'Initial align based on inertial frame.');
    G = diag([cos(pos(1)),sin(pos(1))*cos(pos(1)),cos(pos(1))^2]);
    Mk = zeros(3); Fk = zeros(1,3);
    for k=1:nn:len-nn+1
        wvm = imu(k:k+nn-1, 1:6);  kts = (k+nn-1)*ts;
        swiet = sin(kts*glv.wie); cwiet = cos(kts*glv.wie);
        [phim, dvbm] = cnscl(wvm);
        fib0 = qmulv(qib0b, dvbm)/nts;   % f
        vib0 = vib0 + fib0*nts;          % vel
        pib0 = ratio*pib0 + (vib0_1+vib0)*nts/2;  vib0_1 = vib0; % pos
        [fi0, vi0, pi0] = i0fvp(kts, lat);
        qib0b = qupdt(qib0b, phim);  % qib0b updating
        pib0k(ki,:) = pib0'; vib0k(ki,:) = vib0'; fib0k(ki,:) = fib0'; % recording
        pi0k(ki,:) = pi0';   vi0k(ki,:) = vi0';   fi0k(ki,:) = fi0';
        Wk = [swiet; 1-cwiet; -(1-cwiet)];
        Mk = Mk + Wk*fib0';
        Fk = Fk + fib0';
        if k>k0
            k1 = fix(ki/2);
            vi = vib0k(k1,:)'; vj = vib0k(ki,:)'-vi;
            L = sin(1/2*acos(vi'*vj/norm(vi)/norm(vj)))/sin(glv.wie*kts/4);
            if abs(L)<1, L=acos(L); else, L=0; end
            sl = sin(L); cl = cos(L);
            Cnn0 = [cwiet,swiet*sl,-swiet*cl; 
                    -swiet*sl,1-(1-cwiet)*sl^2,(1-cwiet)*sl*cl; 
                    swiet*cl,(1-cwiet)*sl*cl,1-(1-cwiet)*cl^2];
            G = diag([cl,sl*cl,cl^2]);
            Ak = G*Mk+[0;0;1]*Fk;
            Ak1 = Ak; Ak(2,:)=-Ak(2,:);
            [C,q,J] = svdest(Ak);  [C1,q,J1] = svdest(Ak1);
            if J>J1, dC=1; else dC=-1; C=C1; end
            
%             [u,s,v] = svd(Ak);  C = u*v';  dC=det(C); N = diag([1,dC,1]); C = N*C;
            
            L = dC*L;
            sl = sin(L); cl = cos(L);
            Cnn0 = [cwiet,    swiet*sl,        -swiet*cl; 
                    -swiet*sl,1-(1-cwiet)*sl^2,(1-cwiet)*sl*cl; 
                    swiet*cl, (1-cwiet)*sl*cl, 1-(1-cwiet)*cl^2];

            qnb = qmul(qmul(m2qua(Cnn0),m2qua(C)),qib0b);
            attkv(ki,:) = q2att(qnb)';                        % using vel with lat unknown
            Cni0 = [-swiet,cwiet,0; 
                -eth.sl*cwiet,-eth.sl*swiet,eth.cl; 
                eth.cl*cwiet,eth.cl*swiet,eth.sl];
            qi0ib0 = dv2atti(pi0k(k1,:)', pi0, pib0k(k1,:)', pib0);
            qnb = qmul(qmul(m2qua(Cni0),qi0ib0),qib0b);
            attk(ki,:) = q2att(qnb)';     % using pos
            Lk(ki,:) = [L, pos(1), max(max(C-C1)), kts];
       end
       ki = timebar;
    end
    k0 = fix(k0/nn)+1;
%     attk(1:k0,:) = repmat(attk(k0+1,:),k0,1);
    Cni0 = [0,1,0; -eth.sl,0,eth.cl;  eth.cl,0,eth.sl];
    att0 = q2att(qmul(m2qua(Cni0),qi0ib0));
    attk(1:k0,:) = repmat(att0',k0,1);
    attkv(1:k0,:) = repmat(attkv(k0+1,:),k0,1);
    tk = imu(nn:nn:length(attk)*nn,7); attk(:,4) = tk; attkv(:,4) = tk;
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
        hold on,  plot(t, attkv(:,3)/glv.deg, 'm:'), legend('i0 pos', 'i0 vel'); title(sprintf('\\psi=%.4f', attk(end,3)/glv.deg));
