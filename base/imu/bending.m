function [en, A, dphi, TT, str, att] = bending(imu, t1, t2, pos, dK, U, W)
% See also  bendingH, bendingplot.
global glv
    if nargin<5, dK=zeros(24,1); end
    if length(dK)==1, dK=zeros(24,1); end  % dK=0
    if length(dK)==24, [dK,U,W]=bendingX2KUW(dK); end 
    ts = diff(imu(1:2,end));
	att0 = alignsb(datacut(imu,-inf,t1),pos(1),0);
    wnie = [0; cos(pos(1)); sin(pos(1))]*glv.wie;
    qnb = a2qua(att0);  wm_1=imu(1,1:3)'; vm_1=imu(1,4:6)'; att = imu(:,[1:3,end])*0;  att(1,:)=[att0;imu(1,end)]';
    len = length(imu);  phi = [zeros(len,3),imu(:,end)];
    res = [zeros(len,9),imu(:,end)]; 
    ddwf(ts);
    timebar(1, len);
    for k=2:len
        wm = imu(k,1:3)'; vm = imu(k,4:6)'; t = imu(k,end);
        phim = wm+glv.csCompensate*1/12*cross(wm_1,wm); wm_1 = wm; vm_1 = vm;
        [wfdot, wfddot] = ddwf([wm;vm]/ts, 1);
        phim = phim - dK*wm - U*wfdot(4:6)*ts - W*wfddot(1:3)*ts;
%         phim = phim - dK*wm - U*cros(vm,wm/ts) - W*wfddot(1:3)*ts;
        qnb = qupdt2(qnb, phim, wnie*ts);
        att(k,:) = [q2att(qnb); t]';
        res(k,:) = [wfdot; wfddot(1:3); t]';
        timebar;
        if t<t1 || t>t2
            phi(k,:) = aa2phi(att(k,:), att0);
        end
    end
    %%
    phi( phi(:,end)>t1-1 & phi(:,end)<t2+1, :) = [];
    myfig,  t = phi([1;end],end);
    subplot(131),
	[dphiE, p1, p2] = kaafit([phi(:,1)/glv.sec,phi(:,end)], t1, t2, 0);
    plot(phi(:,end), phi(:,1)/glv.sec, t, polyval(p1,t), t,polyval(p2,t)); xygo('phiE');
    subplot(132),
	[dphiN, p1, p2] = kaafit([phi(:,2)/glv.sec,phi(:,end)], t1, t2, 0);
    plot(phi(:,end), phi(:,2)/glv.sec, t, polyval(p1,t), t,polyval(p2,t)); xygo('phiN');
    subplot(133),
	[dphiU, p1, p2] = kaafit([phi(:,3)/glv.sec,phi(:,end)], t1, t2, 0);
    plot(phi(:,end), phi(:,3)/glv.sec, t, polyval(p1,t), t,polyval(p2,t)); xygo('phiUsec');
    %%
    t10=t1+10; t20=t2-10; T=t20-t10;
    att00=datacut(att, t10+30, t20-10);  att00(:,1:3)=delbias(att00(:,1:3));
    Ap = (max(att00(:,1))-min(att00(:,1)))/2;
    Ar = (max(att00(:,2))-min(att00(:,2)))/2;
    Ay = (max(att00(:,3))-min(att00(:,3)))/2; A = [Ap,Ar,Ay];
    pry = [1;2;3]; [mm,idx]=min(A); pry(idx)=[];
    [t_1, tp2np_1, tn2p_1] = zeroyt(att00(:,[pry(1),end]),0);
    [t_2, tp2np_2, tn2p_2] = zeroyt(att00(:,[pry(2),end]),0);
    %%
    if A(pry(1))>A(pry(2))
        TT = tn2p_1(2)-tn2p_1(1);
    else
        TT = tn2p_2(2)-tn2p_2(1);
    end
    en = -[dphiE; dphiN; dphiU]*glv.sec/T;
    dphi = mod((tn2p_1(1)-tn2p_2(1))*2*pi/TT,2*pi);
    Ap = Ap/glv.deg; Ar = Ar/glv.deg; Ay = Ay/glv.deg; dphi = dphi/glv.deg;
    if att(1,2)<45*glv.deg
        if pry(1)==1 && pry(2)==2
            str=sprintf('Ap = %.1f, Ar = %.1f, T = %.1f, pr = %.1f', Ap, Ar, TT, dphi);
            A = [Ap; Ar; 0];  dphi = [0; dphi; 0];
        elseif pry(1)==2 && pry(2)==3
            str=sprintf('Ar = %.1f, Ay = %.1f, T = %.1f, pr = %.1f', Ar, Ay, TT, dphi);
            A = [0; Ar; Ay];  dphi = [0; 0; dphi];
        elseif pry(1)==1 && pry(2)==3
            str=sprintf('Ap = %.1f, Ay = %.1f, T = %.1f, pr = %.1f', Ap, Ay, TT, dphi);
            A = [Ap; 0; Ay];  dphi = [0; 0; dphi];
        end
    else
        if pry(1)==1 && pry(2)==2
            str=sprintf('r90: Ap = %.1f, Ar = %.1f, T = %.1f, pr = %.1f', Ap, Ar, TT, dphi);
            A = [Ap; Ar; 0];  dphi = [0; dphi; 0];
        elseif pry(1)==2 && pry(2)==3
            str=sprintf('r90: Ar = %.1f, Ay = %.1f, T = %.1f, pr = %.1f', Ar, Ay, TT, dphi);
            A = [0; Ar; Ay];  dphi = [0; 0; dphi];
        elseif pry(1)==1 && pry(2)==3
            str=sprintf('r90: Ap = %.1f, Ay = %.1f, T = %.1f, pr = %.1f', Ap, Ay, TT, dphi);           
            A = [Ap; 0; Ay];  dphi = [0; 0; dphi];
        end
    end
	if att(1,1)>45*glv.deg       
        t10=t1+10; t20=t2-10; T=t20-t10;
        att00=datacut(imu, t10+30, t20-10);
        [t_1, tp2np_1, tn2p_1] = zeroyt(att00(:,[1,end]),0);
        [t_2, tp2np_2, tn2p_2] = zeroyt(att00(:,[2,end]),0);
        TT = tn2p_1(2)-tn2p_1(1);
        dphi = mod((tn2p_1(1)-tn2p_2(1))*2*pi/TT,2*pi);
        Ap = 10*glv.deg; Ar = 10*glv.deg; Ay = 0;
        Ap = Ap/glv.deg; Ar = Ar/glv.deg; Ay = Ay/glv.deg; dphi = dphi/glv.deg;
        
        str=sprintf('p90: Ap = %.1f, Ar = %.1f, T = %.1f, pr = %.1f', Ap, Ar, TT, dphi);
        A = [Ap; Ar; 0];  dphi = [0; dphi; 0];
    elseif att(1,1)<-45*glv.deg
        t10=t1+10; t20=t2-10; T=t20-t10;
        att00=datacut(imu, t10+30, t20-10);
        [t_1, tp2np_1, tn2p_1] = zeroyt(att00(:,[1,end]),0);
        [t_2, tp2np_2, tn2p_2] = zeroyt(att00(:,[2,end]),0);
        TT = tn2p_1(2)-tn2p_1(1);
        dphi = mod((tn2p_1(1)-tn2p_2(1))*2*pi/TT,2*pi);
        Ap = 10*glv.deg; Ar = 10*glv.deg; Ay = 0;
        Ap = Ap/glv.deg; Ar = Ar/glv.deg; Ay = Ay/glv.deg; dphi = dphi/glv.deg;
        
        str=sprintf('p-90: Ap = %.1f, Ar = %.1f, T = %.1f, pr = %.1f', Ap, Ar, TT, dphi);
        A = [Ap; Ar; 0];  dphi = [0; dphi; 0];
    end
    subplot(132), title(str);
    A = A*glv.deg; dphi = dphi*glv.deg;


