% Error Distribution Method and Analysis of Observability Degree Based on the Covariances in Kalman Filter.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 2/03/2018, 16/05/2021
glvs
avp0 = avpset([0;0;0], [30;108;380]);  % init AVP
ts = 0.1; T = 600;
%%
twopos = 2;  % =1 for single-position; =2 for two-position alignment
if twopos==1
    imu = imustatic(avp0, ts, T);
elseif twopos==2
    xxx = [];
    seg = trjsegment(xxx, 'init',         0);
    seg = trjsegment(seg, 'uniform',      T/2-10);
    seg = trjsegment(seg, 'turnleft',     20, 9);
    seg = trjsegment(seg, 'uniform',      T/2-10);
    trj = trjsimu(avp0, seg.wat, ts, 1);
    imu = trj.imu;
end
%%
phi = [.1;.1;1]*glv.deg;
imuerr = imuerrset(0.03, 100, 0.001, 10);
wvn = [0.01;0.01;0.01];
[att0v, attkv, xkpk, kfs] = alignvn_kfs(imuadderr(imu,imuerr), qaddphi(a2qua(avp0(1:3)),phi), avp0(7:9), phi, imuerr, wvn);
myfig;  %  Observability plot
spk = sqrt(xkpk(:,13:end-1));  t = xkpk(:,end);
for k=1:12, spk(:,k)=spk(1,k)./spk(:,k); end
subplot(221), semilogy(t, spk(:,1:3),'linewidth',2); title('( a )'); xygo('Observibility'); legend('\phi_E', '\phi_N', '\phi_U')
subplot(222), semilogy(t, spk(:,4:6),'linewidth',2); title('( b )'); xygo('Observibility'); legend('\deltav^n_E', '\deltav^n_N', '\deltav^n_U')
subplot(223), semilogy(t, spk(:,7:9),'linewidth',2); title('( c )'); xygo('Observibility'); legend('\epsilon^b_x', '\epsilon^b_y', '\epsilon^b_z')
subplot(224), semilogy(t, spk(:,10:12),'linewidth',2); title('( d )'); xygo('Observibility'); legend('\nabla^b_x', '\nabla^b_y', '\nabla^b_z')


