% Figure for C++ processing results
% Make sure Matlab/PSINS Toolbox have been initialized!
glvs
PSINSDemo = 6;
switch PSINSDemo
    case -1, %% Demo_SINS/GNSS
        ins = binfile('ins.bin', 16+3);
        insplot(ins(:,[1:16]));
        avpcmpplot(adddt(ins(:,[17:19,16]),0.002), ins(:,[1:3,16]), 'a', 'mu');
        psinstypedef(196);
        [xk,pk,zk,rk,sk] = kffile('kf.bin', 19,6);
        kfplot(xk,pk,1:19);
        stateplot(sk);
    case 1, %% Demo_CIIRV3
        res = binfile('res.bin', 6);
        figure,
        subplot(131), plot(res(:,1:3:6)); grid on;
        subplot(132), plot(res(:,2:3:6)); grid on;
        subplot(133), plot(res(:,3:3:6)); grid on;
    case 2, %% Demo_CMaxMin
        res = binfile('res.bin', 5);
        figure,
        plot(res); grid on; legend('x', 'maxRes', 'minRes', 'maxpreRes', 'minpreRes');
    case 3, %% Demo_CVAR
        res = binfile('res.bin', 3);
        figure,
        plot(res); grid on; legend('x', 'mean', 'var');
    case 4, %% Demo_CVARn
        res = binfile('res.bin', 3);
        figure,
        plot(res); grid on; legend('x', 'meanx', 'stdx');
    case 5, %% Demo_CRAvar
        res = binfile('res.bin', 2);
        figure,
        plot(res); grid on; legend('x', 'sqrt(RAvar)');
    case 6, %% Demo_CSINS_static
        avp = binfile('ins.bin', 16);
        insplot(avp(:,[1:9,end]));
    case 7, %% Demo_CAlignsv
        att = binfile('aln.bin', 4);
        T1 = find(diff(att(:,end))<0,1);
        insplot(att(T1+1:end,:),'a');
        subplot(211), plot(att(1:T1,end), att(1:T1,1:2)/glv.deg, 'r');
        subplot(212), plot(att(1:T1,end), att(1:T1,3)/glv.deg, 'r');
        legend('Fine align', 'Coarse align');
    case 8, %% Demo_CAligntf
        dd = binfile('aln.bin', 25);
        avp = dd(:,1:16); avpr = dd(:,[17:end,16]);
        insplot(avp);
        avpcmpplot(avpr, avp, 'mu');
        psinstypedef(196);
        [xk, pk, zk, rk, sk] = kffile('kf.bin', 19,6, 0);
        kfplot(xk,pk);
        xpplot(xk,pk,7:9,glv.min,'mu');
        xpplot(zk,rk,1:3,1,'dvn');
        xpplot(zk,rk,4:6,glv.min,'datt');
        stateplot(sk);
    case 9, %% Demo_CAlign_CSINS
        att = binfile('aln.bin', 4);
        insplot(att,'a');
        avp = binfile('ins.bin', 16);
        insplot(avp);
    case 10, %% Demo_CSINSGNSS
        res = binfile([glv.rootpath,'\vc60\Data\ins.bin'], 16+6);
        avp = res(:,1:16); gps = no0(res(:,[17:end,16]),1);
        insplot(avp);
        gpsplot(gps);
        avpcmpplot(avp(:,[4:9,end]), gps, 'vp');
        psinstypedef(196);
        [xk,pk,zk,rk,sk] = kffile([glv.rootpath,'\vc60\Data\kf.bin'], 19,6);
        kfplot(xk,pk,1:19);
        rvpplot(zk,rk);
        stateplot(sk);
    case 11, %% Demo_CVCFileFind
        NA = 0;
    case 14,
        [imu, mag, bar, avp, gps, gs, temp] = psinsboardbin(2);
        imuplot(imu);  magplot(mag);  baroplot(bar);
        if ~isempty(gps), gpsplot(gps); end
        templot(temp);
end


