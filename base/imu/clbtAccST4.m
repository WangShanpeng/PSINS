function [Cba, Ka, Ka2, db, gS] = clbtAccST4(Acc, g0)
global glv
    if nargin<2, g0=glv.g0; end
    len = size(Acc,1); for k=1:len, Acc(k,1:3) = Acc(k,1:3)./norm(Acc(k,1:3))*g0;  end
    A = [Acc(:,1:3), ones(len,1)];
    X = lscov(A, Acc(:,4));
    Ka = norm(X(1:3));  db = X(4);
    Cba = X(1:3)/Ka;
%     idx = Ka*Acc(:,4)-db>0;
%     kp = sum((Acc( idx,1:3)*Cba+db))/sum(Acc( idx,4));
%     kn = sum((Acc(~idx,1:3)*Cba+db))/sum(Acc(~idx,4));
%     Ka = (kp+kn)/2;
%     Ka2 = (kp-kn)/2;
%     f = Ka*Acc(:,4) - db;
%     f = Ka*Acc(:,4) - db - Ka2*abs(f);
%     myfig, plot((f-Acc(:,1:3)*Cba)/glv.ug); xygo('k', 'Err / ug');
    f = Acc(:,1:3)*Cba+db;
    idx = f>0; fp=f(idx); fn=f(~idx);
    kp = sum(fp.*fp)/sum(fp.*Acc( idx,4));
    kn = sum(fn.*fn)/sum(fn.*Acc(~idx,4));
    Ka = (kp+kn)/2;
    Ka2 = (kp-kn)/2;
    f = Ka*Acc(:,4) - db;
    f = Ka*Acc(:,4) - db - Ka2*abs(f);
    myfig, plot((f-Acc(:,1:3)*Cba)/glv.ug); xygo('k', 'Err / ug');
    


