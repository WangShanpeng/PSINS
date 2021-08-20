function r = roots4(p)
% https://baike.baidu.com/item/一元四次方程求根公式/10721996?fr=aladdin

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    a = p(1); b = p(2); c = p(3); d = p(4); e = p(5);
    D = 3*b^2 - 8*a*c;
    E = -b^3 + 4*a*b*c - 8*a^2*d;
    F = 3*b^4 + 16*a^2*c^2 - 16*a*b^2*c + 16*a^2*b*d - 64*a^3*e;
    A = D^2 - 3*F;
    B = D*F - 9*E^2;
    C = F^2 - 3*D*E^2;
    delta = B^2 - 4*A*C;
%     if ~(delta<-1e-15 && D>1e-15 && F>1e-15);
%         error('error');
%     end
    if abs(E)>1e-10
        theta = acos((3*B-2*A*D)/(2*A*sqrt(A)));
        y1 = (D-2*sqrt(A)*cos(theta/3))/3;
        y2 = (D+sqrt(A)*(cos(theta/3)+sqrt(3)*sin(theta/3)))/3;
        y3 = (D+sqrt(A)*(cos(theta/3)-sqrt(3)*sin(theta/3)))/3;
        x1 = (-b+sign(E)*sqrt(y1)+(sqrt(y2)+sqrt(y3)))/(4*a);
        x2 = (-b+sign(E)*sqrt(y1)-(sqrt(y2)+sqrt(y3)))/(4*a);
        x3 = (-b-sign(E)*sqrt(y1)+(sqrt(y2)-sqrt(y3)))/(4*a);
        x4 = (-b-sign(E)*sqrt(y1)-(sqrt(y2)-sqrt(y3)))/(4*a);
    else
        x1 = (-b+sqrt(D+2*sqrt(F)))/(4*a);
        x2 = (-b-sqrt(D+2*sqrt(F)))/(4*a);
        x3 = (-b+sqrt(D-2*sqrt(F)))/(4*a);
        x4 = (-b-sqrt(D-2*sqrt(F)))/(4*a);
    end
    r = sort([x1; x2; x3; x4], 1, 'descend');

