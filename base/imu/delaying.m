function [phi, att] = delaying(imu, t1, t2, pos)
% See also  bending.
global glv
    ts = diff(imu(1:2,end));
	att0 = alignsb(datacut(imu,-inf,t1),pos(1),0);
    wnie = [0; cos(pos(1)); sin(pos(1))]*glv.wie;
    qnb = a2qua(att0);  wm_1=imu(1,1:3)'; att = imu(:,[1:3,end])*0;  att(1,:)=[att0;imu(1,end)]';
    vm_1=imu(1,4:6)';
    len = length(imu);  phi = [zeros(len,3),imu(:,end)];
    timebar(1, len);
    for k=2:len
        wm = imu(k,1:3)'; vm = imu(k,4:6)'; t = imu(k,end);
        phim = wm+glv.csCompensate*1/12*cross(wm_1,wm); wm_1 = wm;
        qnb = qupdt2(qnb, phim, wnie*ts);
        att(k,:) = [q2att(qnb); t]';
        timebar;
        if t<t1 || t>t2
            phi(k,:) = aa2phi(att(k,:), att0);
        end
    end
    %%
    phi( phi(:,end)>t1-1 & phi(:,end)<t2+1, :) = [];
    myfig,  t = phi([1;end],end);
    subplot(234),
	[dphiE, p1, p2] = kaafit([phi(:,1)/glv.sec,phi(:,end)], t1, t2, 0);
    plot(phi(:,end), phi(:,1)/glv.sec, t, polyval(p1,t), t,polyval(p2,t)); xygo('phiE');
    subplot(235),
	[dphiN, p1, p2] = kaafit([phi(:,2)/glv.sec,phi(:,end)], t1, t2, 0);
    plot(phi(:,end), phi(:,2)/glv.sec, t, polyval(p1,t), t,polyval(p2,t)); xygo('phiN');
    subplot(236),
	[dphiU, p1, p2] = kaafit([phi(:,3)/glv.sec,phi(:,end)], t1, t2, 0);
    plot(phi(:,end), phi(:,3)/glv.sec, t, polyval(p1,t), t,polyval(p2,t)); xygo('phiUsec');

    %%
    t10=t1+10; t20=t2-10; T=t20-t10;
    att00=datacut(att, t10, t20);  att00(:,1:3)=delbias(att00(:,1:3));
    Ap = (max(att00(:,1))-min(att00(:,1)))/2;
    Ar = (max(att00(:,2))-min(att00(:,2)))/2;
    Ay = (max(att00(:,3))-min(att00(:,3)))/2; A = [Ap,Ar,Ay];
    [mm,idx]=max(A);
    [t_1, tp2np_1, tn2p_1] = zeroyt(att00(:,[idx,end]),0);
    TT = tn2p_1(2)-tn2p_1(1);    
    subplot(231),    plot(att(:,end), att(:,1)/glv.deg); xygo('p');
    subplot(232),    plot(att(:,end), att(:,2)/glv.deg); xygo('r');  title(sprintf('T = %.2f', TT));
    subplot(233),    plot(att(:,end), att(:,3)/glv.deg); xygo('y');

