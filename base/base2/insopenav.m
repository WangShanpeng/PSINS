function av = insopenav(imu, att0, vp, isfig)
% Process SINS pure inertial navigation with known vel & pos.
%
% Prototype: av = insopenav(imu, att0, vp)
% Inputs: imu - SIMU data array
%         att0 - initial att
%         vp - reference vel & pos.
% Output: av - navigation results, avp = [att,vn,t]
%
% See also  inspure.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/11/2023
global glv
    if size(vp,2)<5
        vn = pp2vn(vp);
        vp = [vn(:,1:3), vp(:,[1:3,end])];
    end
    [nn, ts, nts] = nnts(2, imu(2,end)-imu(1,end));  nts2 = nts/2;
    qnb = a2qua(att0); vn = zeros(3,1);    eth = earth(vp(1,4:6)', vp(1,1:3)');
    len = length(imu);    av = zeros(fix(len/nn), 7);
    ki = timebar(nn, len, 'Pure AV navigation processing.');
    imuvp = combinedata(imu, vp);
    for k=1:nn:len-nn+1
        k1 = k+nn-1;
        wvm = imuvp(k:k1, 1:6);
        [phim, dvbm] = cnscl(wvm,0);
        for k2=k:k1
            if imuvp(k2,11)~=0, eth = earth(imuvp(k2,11:13), imuvp(k2,8:10));  break; end
        end
        fn = qmulv(qnb, dvbm/nts);
        an = rotv(-eth.wnin*nts2, fn) + eth.gcc;
        vn = vn + an*nts;
        qnb = qupdt2(qnb, phim, eth.wnin*nts);
        av(ki,:) = [q2att(qnb); vn; imuvp(k1,7)]';
        ki = timebar;
    end
    if nargin<4, isfig=1; end
    if isfig==1,
        insplot(av,'av');
    end
