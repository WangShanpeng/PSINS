% GHQKG-CKF-UKF filtering common frame, called by 'test_nonlinear_Gauss_KF_vertically_falling_body.m'
Xk = Xk0; Pk = Pk0;
L = length(wm);  Xikk_1 = zeros(n,L);  Zikk_1 = zeros(m,L);
for k=1:len
    sP = chol(Pk,'lower');  Xik_1 = sP*U + repmat(Xk,1,L);
    Xkk_1=zeros(n,1);
    for k1=1:L, Xikk_1(:,k1) = vfbfx(Xik_1(:,k1), tpara);
                Xkk_1 = Xkk_1+wm(k1)*Xikk_1(:,k1);  end
    Pkk_1=Qk;
    for k1=1:L, dX = Xikk_1(:,k1)-Xkk_1; Pkk_1 = Pkk_1+wc(k1)*dX*dX'; end;
    if mod(k,zint)==0
        sP = chol(Pkk_1,'lower');  Xikk_1 = sP*U + repmat(Xkk_1,1,L);
        Zkk_1=zeros(m,1);
        for k1=1:L, Zikk_1(:,k1) = vfbhx(Xikk_1(:,k1), tpara);
                    Zkk_1 = Zkk_1+wm(k1)*Zikk_1(:,k1);  end
        PXZ = zeros(n,m); PZZ = Rk;
        for k1=1:L, dX = Xikk_1(:,k1)-Xkk_1; dZ = Zikk_1(:,k1)-Zkk_1;
                    PXZ = PXZ+wc(k1)*dX*dZ'; PZZ = PZZ+wc(k1)*dZ*dZ';  end;
        Kk = PXZ*PZZ^-1;
        Xk = Xkk_1 + Kk*(zk(k)-Zkk_1);
        Pk = Pkk_1 - Kk*PZZ*Kk';  Pk = (Pk+Pk')/2;
    else
        Xk = Xkk_1;  Pk = Pkk_1;
    end
	res(k,:) = [Xk; diag(Pk)]';
end
