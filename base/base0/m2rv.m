function rv = m2rv(m, rv0)
% Convert transformation matrix to rotation vector.
%
% Prototype: rv = m2rv(m)
% Input: m - transformation matrix
%        rv0 - make sure the direction of output rv same as rv0 
% Output: rv - corresponding rotation vector, such that
%     m = I + sin(|rv|)/|rv|*(rvx) + [1-cos(|rv|)]/|rv|^2*(rvx)^2
%     where rvx is the askew matrix or rv.
% 
% See also  rv2m, rv2q, q2rv, m2att.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/02/2009, 18/03/2014
        rv = [m(3,2)-m(2,3); m(1,3)-m(3,1); m(2,1)-m(1,2)];  % 11/10/2022
        phi = acos((m(1,1)+m(2,2)+m(3,3)-1)/2);
        if phi>pi-1e-3,  % near pi
            rv=q2rv(m2qua(m));
        elseif phi<1e-10,  % very small
            rv=1/2*rv; 
        else
            rv=rv*phi/(2*sin(phi));
        end
        if nargin>1
            nrv = rv'*rv0; if nrv<0;  nrv=norm(rv); rv=-rv/nrv*(2*pi-nrv); end;
        end
        return;
%     rv = iaskew(m-glv.I33);  % coarse init is ok when rv is small, otherwise may fail
%     rvx = askew(rv); % good! the following iteration due to the
    for k=1:2        % accuracy deduce of sqrt(.) in function m2qua
        xx = rv(1)*rv(1); xy = rv(1)*rv(2); xz = rv(1)*rv(3);
        yy = rv(2)*rv(2); yz = rv(2)*rv(3); zz = rv(3)*rv(3);
        n2 = xx+yy+zz;
%         n2 = rv'*rv; 
        if n2>1.0e-40
            n = sqrt(n2);
%             rvx = (m-eye(3)-(1-cos(n))/n2*rvx*rvx) * (n/sin(n));
            rvx = (m1-(1-cos(n))/n2*[-yy-zz,xy,xz; xy,-xx-zz,yz; xz,yz,-xx-yy]) * (n/sin(n));
            rv = [rvx(3,2); rvx(1,3); rvx(2,1)]; % rv = iaskew(rvx);
        else
            rv = zeros(3,1);
            break;
        end
    end
