function [H, Hk, Hu, Hw] = bendingH(A, phi, T, typ, typp)
% See also  bending, bendingplot.
global glv
    if nargin<5, typp=0; end
    if nargin<4, typ=0; end
    if length(phi)<3, phi=[0;phi]; end
    if length(A)<2, A=[A;A;A]; end
    pr = A(1)*A(2)*sin(phi(2)-phi(1));
    py = A(1)*A(3)*sin(phi(3)-phi(1));
    ry = A(2)*A(3)*sin(phi(3)-phi(2));
    w = 2*pi/T;
    Hk = [ ry  -py   pr  0  -ry   0    0   0  -ry
           py   ry   0   0  -py   pr   0   0   py
          -pr   0    ry  0  -pr  -py   0   0   pr ];
    Hu = [ 0   0   0   ry  -py  0   0    pr  0;
          -ry  py  0   0    0   0  -pr   0   0;
           0  -pr  0   pr   0   0   0    0   0 ];
    Hw = [ 0   0   0  -py  -ry  0   pr   0  -ry;
           py  ry  0   0    0   0   0    pr  py;
          -pr  0   ry  0   -pr -py  0    0   0 ];
if typ>0
    Hk = [-ry   0   -pr   0  -ry  -py   0   0   ry
           py  -pr   0    0  -py   ry   0   0   py
           pr   py  -ry   0  -pr   0    0   0  -pr ];
    Hu = [ 0  -pr  0   0  -py   ry  0    0   0;
           0   0   pr  0   0    0   0    py -ry;
           0   0   0   0   0    pr  0   -pr  0 ];
    Hw = [-ry  0  -pr  0   -ry -py  0     0   0;
           py -pr  0   0    0   0   0     ry  py;
           0   0   0   py  -pr  0  -ry    0  -pr ];
end
if typp>0  % pch=90deg
    Hk = [ 0   0   pr   0  0   0   0   0  0
           pr  0   0    0  pr  0   0   0 -pr
           0   0   0    0  0   pr  0   0  0 ];
    Hu = [ 0   0   0   0   0    0   0    0  -pr;
           0   0  -pr  0   0    0   0    0   0;
           0   0   0   0   0    0   0    0   0 ];
    Hw = [ 0   0   0   0   0   0   pr   0   0;
           pr  0   0   0   pr  0   0    0   0;
           0   0   0   0   0   0   0    pr  0 ];
elseif typp<0  % pch=-90deg
    Hk = [ 0   0   pr   0  0   0   0   0  0
          -pr  0   0    0 -pr  0   0   0  pr
           0   0   0    0  0  -pr  0   0  0 ];
    Hu = [ 0   0   0   0   0    0   0    0   pr;
           0   0  -pr  0   0    0   0    0   0;
           0   0   0   0   0    0   0    0   0 ];
    Hw = [ 0   0   0   0   0   0   pr   0   0;
          -pr  0   0   0  -pr  0   0    0   0;
           0   0   0   0   0   0   0   -pr  0 ];
end
%     H = 0.5*w*[ Hk(:,[1:3,5:6,9]), glv.g0*Hu(:,[1:3,5:6,9]), -w^2*Hw(:,[1:3,5:6,9]) ];
    H = 0.5*w*[ Hk(:,[1:3,5:6,9]), glv.g0*Hu, -w^2*Hw ];
return

Hx   = bendingH([10;10; 0]*glv.deg, [10;60]*glv.deg, 4, 0);
Hy   = bendingH([10; 0;10]*glv.deg, [10;60]*glv.deg, 4, 0);
Hz   = bendingH([ 0;10;10]*glv.deg, [10;60]*glv.deg, 4, 0);
Hx1  = bendingH([10;10; 0]*glv.deg, [30;90]*glv.deg, 4, 1);
Hy1  = bendingH([10; 0;10]*glv.deg, [30;90]*glv.deg, 4, 1);
Hz1  = bendingH([ 0;10;10]*glv.deg, [30;90]*glv.deg, 4, 1);
Hx2  = bendingH([10;10; 0]*glv.deg, [20;90]*glv.deg, 2, 0);
Hy2  = bendingH([10; 0;10]*glv.deg, [20;90]*glv.deg, 2, 0);
Hz2  = bendingH([ 0;10;10]*glv.deg, [20;90]*glv.deg, 2, 0);
Hx3  = bendingH([10;10; 0]*glv.deg, [30;90]*glv.deg, 2, 1);
Hy3  = bendingH([10; 0;10]*glv.deg, [30;90]*glv.deg, 2, 1);
Hz3  = bendingH([ 0;10;10]*glv.deg, [30;90]*glv.deg, 2, 1);
Hxp  = bendingH([10;10; 0]*glv.deg, [110;160]*glv.deg, 4, 0, 1)
Hxp1 = bendingH([10;10; 0]*glv.deg, [110;160]*glv.deg, 2, 0, 1)
H = [Hx; Hy; Hz; Hx1; Hy1; Hz1; Hx2; Hy2; Hz2; Hx3; Hy3; Hz3; Hxp; Hxp1];
myfig, [u,s,v] = svd(H); plot(diag(s),'-o'); grid on

syms kxx kxy kxz kyx kyy kyz kzx kzy kzz pr py ry
  a = [   kxz,             -kxy,         -(kyy+kzz)+kxx;
          kyz,              kxx+kzz-kyy,  kxy;
          -(kxx+kyy)+kzz,  -kyz,          kxz  ] * [pr;py;ry];
  b = [ ry  -py   pr  0  -ry   0    0   0  -ry
        py   ry   0   0  -py   pr   0   0   py
       -pr   0    ry  0  -pr  -py   0   0   pr ] * [kxx; kxy; kxz; kyx; kyy; kyz; kzx; kzy; kzz];
  simplify(a-b)
  a = [  -kxz,          -kyz,         -(kyy+kxx)+kzz;
         -kxy,           kzz+kxx-kyy,  kyz;
         -(kzz+kyy)+kxx, kxy,         -kxz  ] * [pr;py;ry];
  b = [ -ry   0   -pr   0  -ry  -py   0   0   ry
         py  -pr   0    0  -py   ry   0   0   py
         pr   py  -ry   0  -pr   0    0   0  -pr ] * [kxx; kxy; kxz; kyx; kyy; kyz; kzx; kzy; kzz];
  simplify(a-b)

syms uxx uxy uxz uyx uyy uyz uzx uzy uzz pr py ry
  a = [ uzy,-uyy,uyx;
       -uzx,uxy,-uxx;
       (-uxy+uyx),0,0 ] * [pr;py;ry];
  b = [ 0   0   0   ry  -py  0   0    pr  0;
       -ry  py  0   0    0   0  -pr   0   0;
        0  -pr  0   pr   0   0   0    0   0 ] * [uxx; uxy; uxz; uyx; uyy; uyz; uzx; uzy; uzz];
  simplify(a-b)
  a = [ -uxy,-uyy,uyz;
         uxz,uzy,-uzz;
         (-uzy+uyz),0,0 ] * [pr;py;ry];
  b = [ 0  -pr  0   0  -py   ry  0    0   0;
        0   0   pr  0   0    0   0    py -ry;
        0   0   0   0   0    pr  0   -pr  0 ] * [uxx; uxy; uxz; uyx; uyy; uyz; uzx; uzy; uzz];
  simplify(a-b)
  
syms wxx wxy wxz wyx wyy wyz wzx wzy wzz pr py ry
  a = [ wzx,      -wyx,     -(wyy+wzz);
        wzy,       wxx+wzz,  wxy;
      -(wxx+wyy), -wyz,      wxz ] * [pr;py;ry];
  b = [ 0   0   0  -py  -ry  0   pr   0  -ry;
        py  ry  0   0    0   0   0    pr  py;
       -pr  0   ry  0   -pr -py  0    0   0 ] * [wxx; wxy; wxz; wyx; wyy; wyz; wzx; wzy; wzz];
  simplify(a-b)
  a = [ -wxz,      -wyz,     -(wxx+wyy);
        -wxy,       wxx+wzz,  wzy;
        -(wyy+wzz), wyx,     -wzx ] * [pr;py;ry];
  b = [ -ry  0  -pr  0  -ry -py  0     0   0;
         py -pr  0  0    0   0   0     ry  py;
         0  0    0  py  -pr  0  -ry    0  -pr ] * [wxx; wxy; wxz; wyx; wyy; wyz; wzx; wzy; wzz];
  simplify(a-b)
    