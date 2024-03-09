function [Sfg0, Sfa0, Cbg0, Cba0, eg0, da0, Kapn0] = clbt5axisGet47(ipara, g0)
global glv
    if nargin<2, g0=9.79406; end
    ip=load(ipara);
    for k=1:3
        Cbg0(k,4) = cos(ip(11));  Cbg0(k,5) = cos(ip(14));
        Cba0(k,4) = cos(ip(40));  Cba0(k,5) = cos(ip(43));
    end
    Cbg0(2,1) = ip(6);
    Cbg0(1,2) = ip(7);
    Cbg0(1,3) = ip(8);
    Cbg0(3,1) = ip(9);
    Cbg0(3,2) = ip(10);
    Cbg0(2,3) = ip(11);
    Cba0(2,3) = ip(38);
    Cba0(1,3) = ip(39);
    Cba0(1,2) = ip(40);
    for k=1:5
        Sfg0(k,1) = ip(k);
        eg0(k,1) = ip(k+17);
        Kap0(k,1) = ip(k+22) / ip(47) * g0;
        Kan0(k,1) = ip(k+27) / ip(47) * g0;
        da0(k,1) = ip(k+32) / glv.ug;
        Sfa0(k,1) = (Kap0(k)+Kan0(k))/2;
        Kapn0(k,1) = (Kap0(k)-Kan0(k))/(Kap0(k)+Kan0(k)) / glv.ppm;
    end
    Cbg0(1,1)=1.0; Cbg0(2,2)=1.0; Cbg0(3,3)=1.0; 
    Cba0(1,1)=1.0; Cba0(2,2)=1.0; Cba0(3,3)=1.0; 
    