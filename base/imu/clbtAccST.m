function [Cba, Ka, Ka2, db] = clbtAccST(Acc, Cba0, g0)
global glv
    if nargin<2, Cba0=[]; end
    if nargin<3, g0=glv.g0; end
    if isempty(Cba0)
        [m,idx] = max(Acc(:,1));  Cba0(1,1)=Acc(idx,4)/m;
        [m,idx] = max(Acc(:,2));  Cba0(2,1)=Acc(idx,4)/m;
        [m,idx] = max(Acc(:,3));  Cba0(3,1)=Acc(idx,4)/m;  Cba0 = Cba0/norm(Cba0);
    end
    len=length(Acc); for k=1:len, Acc(k,1:3) = Acc(k,1:3)./norm(Acc(k,1:3))*g0;  end
    [Cba, Ka, Ka2, db] = clbtAccST0(Acc, Cba0, g0);
    a = [-Cba(2); Cba(1); 0];  a = a/norm(a);
    b = cross(Cba, a);
    A = abs([Acc(:,1:3)*Cba, Acc(:,1:3)*a, Acc(:,1:3)*b]).^2;
    X = [0;0;0];
    for k=1:5
        f = Ka*Acc(:,4);
        f = f - db - Ka2*abs(f) - abs(Acc(:,1:3)*Cba).^2*X(1)-abs(Acc(:,1:3)*a).^2*X(2)-abs(Acc(:,1:3)*b).^2*X(3);
        err = f-Acc(:,1:3)*Cba;
        dX = lscov(A, err);
        X = X+dX;
    end
    myfig, 
    subplot(211), plot([Acc(:,1:3),Acc(:,1:3)*Cba]/g0); xygo('k', 'Acc / g');
    subplot(212), plot(err/glv.ug); xygo('k', 'Err / ug');
    return
%   X(0)=Cba0[0][aST-1], X(1)=Cba0[1][aST-1], X(2)=Cba0[2][aST-1], X(3)=1.0, X(4)=X(5)=0.0;
% 	A.SetClm(0,wf5.GetClm(5)); A.SetClm(1,wf5.GetClm(6)); A.SetClm(2,wf5.GetClm(7)); A.SetClm(5,Onen1);
% 	A.SetClm(3,-wf5.GetClm(4+aST)); A.SetClm(4,abs(wf5.GetClm(4+aST)));  // for Kapn
% //	A.SetClm(3,-wf5.GetClm(4+aST)); A.SetClm(4,pow(wf5.GetClm(4+aST),2));  // for Ka2
% 	for(int k=0; k<5; k++) {
% 		A(19,0) = 2.0*X(0), A(19,1) = 2.0*X(1), A(19,2) = 2.0*X(2),  A(19,3)=A(19,4)=A(19,5)=0.0;
% 		Y = -(A*X); Y(19)=0.0;
% 		dX = lss(A, Y);
% 		X += dX;
% 		normlize((CVect3*)&X.dd[0]);  // X=[cx,cy,cz,dka,ka2,db]
% 	}
function [Cba, Ka, Ka2, db] = clbtAccST0(Acc, Cba0, g0)
global glv
    if nargin<2, Cba0=[]; end
    if nargin<3, g0=glv.g0; end
    if isempty(Cba0)
        [m,idx] = max(Acc(:,1));  Cba0(1,1)=Acc(idx,4)/m;
        [m,idx] = max(Acc(:,2));  Cba0(2,1)=Acc(idx,4)/m;
        [m,idx] = max(Acc(:,3));  Cba0(3,1)=Acc(idx,4)/m;  Cba0 = Cba0/norm(Cba0);
    end
    len = size(Acc,1);
    for k=1:len, Acc(k,1:3) = Acc(k,1:3)./norm(Acc(k,1:3))*g0;  end
    X = [Cba0; 1; 0; 0];
    A = [Acc(:,1:3), -Acc(:,4), abs(Acc(:,4)), ones(len,1)];
    W = ones(len+1,1); % W = 1:len+1;  W(16:end)=1000;
    for k=0:5
        A(len+1,:) = [2*X(1:3); 0; 0; 0]';
        Y = -(A*X); Y(end)=0;
        dX = lscov(A, Y, W);
        X = X+dX;
        X(1:3) = X(1:3)/norm(X(1:3));
    end
    Cba = X(1:3); Ka = X(4); Ka2 = X(5); db = X(6); 
	f = Ka*Acc(:,4);
	f = f - db - Ka2*abs(f);
	err = f - Acc(:,1:3)*Cba;
    myfig, 
    subplot(211), plot([Acc(:,1:3),Acc(:,1:3)*Cba]/g0, '-o'); xygo('k', 'Acc / g');
    subplot(212), plot(err/glv.ug); xygo('k', 'Err / ug');
    return
