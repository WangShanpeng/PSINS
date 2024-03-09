function [H, Hk, Hu, Hw] = bendingH(A, phi, T, typ)
% See also  bending, bendingplot.
global glv
    if nargin<4, typ=0; end
    if length(phi)<3, phi=[0;phi]; end
    if length(A)<2, A=[A;A;A]; end
    pr = A(1)*A(2)*sin(phi(2)-phi(1));
    py = A(1)*A(3)*sin(phi(3)-phi(1));
    ry = A(2)*A(3)*sin(phi(3)-phi(2));
    w = 2*pi/T;
    Hk = [ ry  -py   pr  0  -ry   0    0   0  -ry
           py   ry   0   0  -py   pr   0   0   py
          -pr   0    ry  0  -pr  -py   0   0  -pr ];
    Hu = [ 0   0   0   ry  -py  0   0    pr  0;
          -ry  py  0   0    0   0  -pr   0   0;
           0  -pr  0   pr   0   0   0    0   0 ];
    Hw = [ 0   0   0  -py  -ry  0   pr   0  -ry;
           py  ry  0   0    0   0   0    pr  py;
          -pr  0   ry  0   -pr -py  0    0   0 ];
if typ==1
    Hk = [-ry   0   -pr   0  -ry  -py   0   0   ry
           py  -pr   0    0  -py   ry   0   0   py
           pr   py  -ry   0  -pr   0    0   0  -pr ];
    Hu = [ 0  -pr  0   0  -py   ry  0    0   0;
           0   0   pr  0   0    0   0    py -ry;
           0   0   0   0   0    pr  0   -pr  0 ];
    Hw = [-pr  0  -pr  0  -py -ry  0     0   0;
           py -pr  0  0    0   0   0     ry  py;
           0  0    0  py  -ry  0  -ry    0  -pr ];    
end
    H = 0.5*w*[ Hk(:,[1:3,5:6,9]), glv.g0*Hu, -w^2*Hw ];
