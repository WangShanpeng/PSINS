function [Cba, Ka, Ka2, db, gS] = clbtAccSTiop(Acc, Cba0, g0, ang)
global glv
    if nargin<2, Cba0=[]; end
    if nargin<3, g0=glv.g0; end
    if nargin<4, ang=0*glv.deg; end
    if isempty(Cba0)
        [m,idx] = max(Acc(:,1));  Cba0(1,1)=Acc(idx,4)/m;
        [m,idx] = max(Acc(:,2));  Cba0(2,1)=Acc(idx,4)/m;
        [m,idx] = max(Acc(:,3));  Cba0(3,1)=Acc(idx,4)/m;  Cba0 = Cba0/norm(Cba0);
    end
    len = size(Acc,1);
    for k=1:len, Acc(k,1:3) = Acc(k,1:3)./norm(Acc(k,1:3))*g0;  end
%     X = [Cba0; 1; 0; 0; 0;0;0];  Cba=Cba0;
    X = [Cba0; 1; 0; 0; 0;0;0;0];  Cba=Cba0;
    W = ones(len+1,1); W(19:end)=3;
    for k=0:10
        a = [-Cba(2); Cba(1); 0];  a = a/norm(a);
        b = cross(Cba, a);
        rv = Cba*ang; a = rotv(rv, a);  b = rotv(rv, b);
%         A = [Acc(:,1:3), -Acc(:,4), abs(Acc(:,4)), ones(len,1), abs([Acc(:,1:3)*Cba, Acc(:,1:3)*a, Acc(:,1:3)*b]).^2];
%         A(len+1,:) = [2*X(1:3); 0; 0; 0; 0;0;0]';
        ai = Acc(:,1:3)*Cba; ao = Acc(:,1:3)*a; ap = Acc(:,1:3)*b;
        A = [Acc(:,1:3), -Acc(:,4), abs(Acc(:,4)), ones(len,1), ai.*ao, ai.*ap, ao.*ao, ap.*ap];
        A(len+1,:) = [2*X(1:3); 0; 0; 0; 0;0;0;0]';
        Y = -(A*X); Y(end)=0;
        dX = lscov(A, Y, W);
        X = X+dX;
        X(1:3) = X(1:3)/norm(X(1:3));
        Cba = X(1:3);
    end
    Cba = X(1:3); Ka = X(4); Ka2 = X(5); db = X(6);  gS = X(7:end);
	f = Ka*Acc(:,4);
% 	f = f - Ka2*abs(f) - db - [abs([Acc(:,1:3)*Cba, Acc(:,1:3)*a, Acc(:,1:3)*b])].^2*X(7:9);
	f = f - Ka2*abs(f) - db - [ai.*ao, ai.*ap, ao.*ao, ap.*ap]*gS;
	err = f - Acc(:,1:3)*Cba;
    myfig, 
    subplot(211), plot([Acc(:,1:3),Acc(:,1:3)*Cba]/g0, '-o'); xygo('k', 'Acc / g');
    subplot(212), plot(err/g0); xygo('k', 'Err / g');
    return
