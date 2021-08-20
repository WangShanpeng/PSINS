% Transfer align simulation using real FOG/MEMS data under vehicular motion.
% See also  test_align_transfer.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/01/2021
glvs
psinstypedef('test_align_transfer_fog_mems_def');
load([glv.datapath,'transfer_align_fog_mems.mat']); % imuplot(mimu); insplot(fogavp);
imuavp = combinedata(mimu, fogavp);
ts = diff(mimu(1:2,end));
imuerr = imuerrset(500, 1000, 0.1, 10);  ebdb=zeros(6,1);
[~, ~, beta, Q] = markov2([6;-10;7]/10*glv.min, [.5;.4;10], ts, 1);
qnbs = a2qua(fogavp(1,1:3)'); vns = fogavp(1,4:6)'; % slave MEMS-INS init
eth = earth(fogavp(1,7:9)', fogavp(1,4:6)');
kf = kfinit(ts, imuerr, beta, Q);
len = length(fogavp); [res, xkpk] = prealloc(len, 7, 2*kf.n+1);  kk=1;
timebar(1, len, 'FOG/MIMU transfer alignment simulaton.');
for k=1:length(mimu)
    [phim, dvbm] = cnscl(imuavp(k,1:6)-ebdb'*ts); t = imuavp(k,7);
    Cnbs = q2mat(qnbs);
    dvn = Cnbs*dvbm; vns = vns + dvn + eth.gcc*ts;  % slave INS velocity updating
    qnbs = qupdt(qnbs, phim-Cnbs'*eth.wnin*ts); 	% slave INS attitude updating
    kf.Phikk_1(1:6,1:3) = [-askew(eth.wnin*ts)+glv.I33; askew(dvn)];
        kf.Phikk_1(1:3,7:9) = -Cnbs*ts; kf.Phikk_1(4:6,10:12) = Cnbs*ts;
    kf = kfupdate(kf);
    fogavpk = imuavp(k,8:16)';
    if fogavpk(7)>0.1   % master FOG-INS is valid
        qnbm = a2qua(fogavpk(1:3));  vnm = fogavpk(4:6);  posm = fogavpk(7:9);
        eth = earth(posm, vnm);
        kf.Hk(1:3,13:18) = [-Cnbs,-Cnbs];
        kf = kfupdate(kf, [qq2phi(qnbs,qnbm); vns-vnm], 'M');
%        qnbs = qdelphi(qnbs, kf.xk(1:3));  kf.xk(1:3)=0;  % feedback
        ebdb = ebdb + kf.xk(7:12);  kf.xk(7:12)=0;
        res(kk,:) = [qq2phi(qmul(qnbs,rv2q(-mu0)),qnbm); vns-vnm; t]; % record
        xk = kf.xk; xk(7:12) = ebdb;
        xkpk(kk,:) = [xk; diag(kf.Pxk); t]';  kk=kk+1;
    end
    timebar;
end
res(kk:end,:)=[];  xkpk(kk:end,:)=[];
kfplot(xkpk, res, eb0, db0, mu0);



