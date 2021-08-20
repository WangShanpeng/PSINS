function [phi, iter] = btzpicard(Wt, T)
% see also qpicard, rodpicard.
    phi = [0;0;0];
    for iter=1:6
        pw = polycross(phi,Wt);
        ppw = polycross(phi,pw);
        phi2 = polydot(phi,phi);  phi4 = conv(phi2,phi2);   phi6 = conv(phi2,phi4);
        f = polyadd(1/12, 1/720*phi2, 1/30240*phi4, 1/1209600*phi6);
        phi = polycut(polyintn(polyadd(Wt, 0.5*pw, polydotmul(f,ppw))),20);
    end
    phi = polyvaln(phi, T);

