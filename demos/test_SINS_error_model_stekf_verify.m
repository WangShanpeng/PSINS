% SINS STEKF-linear-error-model propagation accuracy verification.
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_error_model_verify, alignvn_stekf.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/06/2022
glvs
trj = trjfile('trj10ms.mat');
% initial settings
psinstypedef(156);
[nn, ts, nts] = nnts(2, trj.ts);
imuerr = imuerrset(.1, 1000, 0, 0);
imu = imuadderr(trj.imu, imuerr);
davp0 = avperrset([10;-10;600]/10, [1;2;3]*0, [10;20;30]);
ins = insinit(avpadderr(trj.avp0,davp0), ts);
xk = [davp0; imuerr.eb; imuerr.db];  stxk = xk; % init state error
len = length(imu); [avp, xkk, zkk] = prealloc(fix(len/nn), 10, 10, 7);  stxkk=xkk; stzkk=zkk;
ki = 1;
tbstep = floor(len/nn/100); tbi = timebar(1, 99, 'SINS STEKF linear error model verifies.');
for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);    ins = inslever(ins);
    [Fk, Ft] = kffk(ins); Hk = kfhk(ins); % Hk(1:3,1:3) = -askew(ins.vn); % ??
    xk = expm(Ft*nts)*xk; zk = Hk*xk;
    % stekf error model  - etmST
    Ft = etmST(ins);
    Hk(1:3,1:3) = askew(ins.vn);
    stxk = expm(Ft*nts)*stxk; stzk = Hk*stxk;  %  stxk(9)=xk(9);  % stxk(1:3)=xk(1:3);  
    %
    avp(ki,:) = [ins.avp', t];
    xkk(ki,:) = [xk(1:9)', t];  stxkk(ki,:) = [stxk(1:9)', t];
    zkk(ki,:) = [zk', t];  stzkk(ki,:) = [stzk', t];          ki = ki+1;
    if mod(tbi,tbstep)==0, timebar; end;  tbi = tbi+1;
end
% insplot(avp);
avperr = avpcmp(avp, trj.avp);
inserrplot(avperr);
subplot(221), hold on, plot(xkk(:,end), xkk(:,1:2)/glv.sec, 'm:', stxkk(:,end), stxkk(:,1:2)/glv.sec, 'g:');
subplot(222), hold on, plot(xkk(:,end), xkk(:,3)/glv.min, 'm:',   stxkk(:,end), stxkk(:,3)/glv.min, 'g:');
subplot(223), hold on, plot(xkk(:,end), xkk(:,4:6), 'm:',         stxkk(:,end), stxkk(:,4:6), 'g:');
subplot(223), hold on, plot(xkk(:,end), zkk(:,1:3), 'm-.',         stxkk(:,end), stzkk(:,1:3), 'g-.');
subplot(224), hold on, plot(xkk(:,end), [xkk(:,7:8)*glv.Re,xkk(:,9)], 'm:',  stxkk(:,end), [stxkk(:,7:8)*glv.Re,stxkk(:,9)], 'g:');
stxkk1=stxkk; stxkk1(:,4:6) = stzkk(:,1:3);
avpcmpplot(xkk, stxkk1, 'avp');


