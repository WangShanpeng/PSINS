function gn = egm(GM, Re, wie, f, C, S, lat, lon, hgt)
% Yangongmin, 11/06/2016
    N = length(C)-1;  % order
    C = [C(:,1)*sqrt(2),C(:,2:end)*2];   S = [S(:,1)*sqrt(2),S(:,2:end)*2];
    slat = sin(lat);  clat = cos(lat);  slon = sin(lon);  clon = cos(lon);
    e2 = 2*f-f^2;  RN = Re/sqrt(1-e2*slat^2);  sf=1.0e300;
    x = (RN+hgt)*clat*clon;  y = (RN+hgt)*clat*slon;  z = (RN*(1-e2)+hgt)*slat;
    phi = atan2(z,sqrt(x^2+y^2));  r = sqrt(x^2+y^2+z^2);  dL = lat-phi;
    theta = pi/2-phi;  sth = sin(theta);  cth = cos(theta);
    Pnm = zeros(1,N+1); Pn_2m = Pnm; Pn_1m = Pnm; dPnm = Pnm;
    P = zeros(N+1); dP = P;
    for n=0:N  % legendre
        n1 = n+1;
        if n==0,
            Pnm(n1) = 1/sqrt(2)*sf;
            dPnm(n1) = 0;
        else
            Pnm(n1) = sqrt((2*n+1)/(2*n))*Pn_1m(n1-1)*sth;  % (3.3.97a)
            Pnm(n1-1) = sqrt(2*n+1)*cth*Pn_1m(n1-1);  % (3.3.97b)
            if n>=2
                m = 0:n-2; k = 1:n-1;
                Pnm(k) = sqrt((2*n+1)./(n^2-m.^2)) .* ...
                    (sqrt(2*n-1)*cth*Pn_1m(k) - ...
                    sqrt(((n-1)^2-m.^2)./(2*n-3)).*Pn_2m(k));  % (3.3.97c)
            end
            dPnm(n1) = n*cth/sth*Pnm(n1);  % 3rd line after (3.3.101)
            m = 0:n-1; k = 1:n;
            dPnm(k) = n*cth/sth*Pnm(k) - ...
                sqrt((2*n+1)/(2*n-1)/sth^2*(n^2-m.^2)).*Pn_1m(k);  % 3rd line after (3.3.101)
        end
        P(n1,:) = Pnm;
        dP(n1,:) = dPnm;
        Pn_2m = Pn_1m; Pn_1m = Pnm;
    end
    N0 = (0:N)';
    Rr = (Re/r).^N0;  % (Re/r)^n
    clon = cos(N0*lon)';   slon = sin(N0*lon)';
    CP = C.*P;  SP = S.*P;
    xx = -Rr*(N0'.*slon).*CP + Rr*(N0'.*clon).*SP;
    yy = (Rr*clon.*C + Rr*slon.*S).*dP;
    zz = (N0+1).*Rr*clon.*CP + (N0+1).*Rr*slon.*SP;
    gE0 =  GM/r^2/sth*sum(sum(xx))/sf;
    gN0 = -GM/r^2*sum(sum(yy))/sf - wie^2/2*r*sin(2*theta);
    gU0 = -GM/r^2*sum(sum(zz))/sf + wie^2*r*sth^2;
    gn = [gE0, gN0*cos(dL)-gU0*sin(dL), gN0*sin(dL)+gU0*cos(dL)]';

