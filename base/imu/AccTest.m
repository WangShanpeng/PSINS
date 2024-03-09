function [Cba, K0, K1, K2, C] = AccTest(Acc, g0, ang, str)
% Acc = [AccX,AccY,AccZ, AccForTest]
% str = 'ii,oo,pp,io,ip,op,ai,i3';
% See also clbtAccST, clbtAccSTiop, clbtAccST2, clbtAccST4
global glv
    if nargin<2, g0=0; end
    if g0<9.0, g0=mean(normv(Acc(:,1:3))); end
    if nargin<3, ang=0*glv.deg; end
    if nargin<4, str='ii'; end
    [m,idx] = max(Acc(:,1));  Cba0(1,1)=Acc(idx,4)/m;
    [m,idx] = max(Acc(:,2));  Cba0(2,1)=Acc(idx,4)/m;
    [m,idx] = max(Acc(:,3));  Cba0(3,1)=Acc(idx,4)/m;  Cba0 = Cba0/norm(Cba0);  % direct cosine vector
    len = size(Acc,1);
%     for k=1:len, Acc(k,1:3) = Acc(k,1:3)./norm(Acc(k,1:3))*g0;  end  % normalize XYZ
    [a2, n] = str2a2(str, Acc(:,1:3));
    X = [Cba0;0;0;zeros(n,1)];  Cba=X(1:3);
    W = ones(len+1,1); W(19:end)=3;
    for k=1:10
        a = [Cba(2); -Cba(1); 0];  a = a/norm(a);
        b = cross(Cba, a);
        rv = Cba*ang; a = rotv(rv, a);  b = rotv(rv, b);
        ai = Acc(:,1:3)*Cba; ao = Acc(:,1:3)*a; ap = Acc(:,1:3)*b;
        [a2,n,unt,untstr] = str2a2(str, ai, ao, ap);
        A = [Acc(:,1:3), -Acc(:,4), ones(len,1), a2];
        A(len+1,:) = [2*X(1:3); 0; 0; zeros(size(a2,2),1)]';
        Y = -(A*X); Y(end)=0;
        dX = lscov(A, Y, W);
        X = X+dX;
        X(1:3) = X(1:3)/norm(X(1:3));
        Cba = X(1:3);  K1 = X(4); K0 = X(5); K2 = X(6:end);
    end
    C = (A'*A)^-1;
	f = K1*Acc(:,4)-K0;
	f = f - a2*K2;
	err = f - Acc(:,1:3)*Cba;
    myfig, 
%     subplot(211), plot(1:len, [[Acc(:,1:3),f]/g0, normv(Acc(:,1:3))/g0],'-o'); xygo('k', 'Acc / g'); legend('X', 'Y', 'Z', 'Test');
    subplot(211), plotyy(1:len, [Acc(:,1:3),f]/g0, 1:len, (normv(Acc(:,1:3))/g0-1)*1e6); xygo('k', 'Acc / g'); legend('X', 'Y', 'Z', 'Test', '|XYZ|-1(ug)');
    title(sprintf('Cba=%.6f,%.6f,%.6f;  K1=%.6f;  K0=%.2fug', Cba(1),Cba(2),Cba(3),K1,K0/glv.ug));
%     subplot(212), plot([K1*Acc(:,4)-K0-Acc(:,1:3)*Cba,err]/g0); xygo('k', 'Err / g'); legend('linear err', 'full model err');
%     title([str,' (ug/g2) = ',sprintf('%.2f ,  ', K2/glv.ugpg2)]);
    subplot(212), plot(err/g0); xygo('k', 'Err / g');
    title([untstr,' = ',sprintf('%.2f ,  ', K2./unt')]);
    return

function [a2,n,unt,untstr] = str2a2(str, ai, ao, ap)
global glv
    if nargin<3, ao=ai(:,2); ap=ai(:,3); ai=ai(:,1); end
    len = length(str);
    a2 = []; unt = []; untstr = []; n = 0;
    for k=1:3:len
        switch str(k:k+1)
            case 'ii', a2 = [a2, ai.*ai]; unt=[unt,glv.ugpg2]; untstr=[untstr,'ii(ug/g2),'];
            case 'oo', a2 = [a2, ao.*ao]; unt=[unt,glv.ugpg2]; untstr=[untstr,'oo(ug/g2),'];
            case 'pp', a2 = [a2, ap.*ap]; unt=[unt,glv.ugpg2]; untstr=[untstr,'pp(ug/g2),'];
            case 'io', a2 = [a2, ai.*ao]; unt=[unt,glv.ugpg2]; untstr=[untstr,'io(ug/g2),'];
            case 'ip', a2 = [a2, ai.*ap]; unt=[unt,glv.ugpg2]; untstr=[untstr,'ip(ug/g2),'];
            case 'op', a2 = [a2, ao.*ap]; unt=[unt,glv.ugpg2]; untstr=[untstr,'op(ug/g2),'];
            case 'ai', a2 = [a2, abs(ai)]; unt=[unt,glv.ppm]; untstr=[untstr,'ai(ppm),'];  % abs(i)
            case 'i3', a2 = [a2, ai.*ai.*ai]; unt=[unt,glv.ugpg3]; untstr=[untstr,'i3(ug/g3),'];
            otherwise, error(sprintf('no string match - %s',str(k:k+1)));
        end
        n = n+1;
    end