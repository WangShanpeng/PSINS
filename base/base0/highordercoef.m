function hoc = highordercoef(n)
% High order coefficients for multi sub-sample rotation vector compensation
% n = 2,3,4,5,...
% Ref. YanGM. 'A new method to obtain high-order error compensation coefficients for equivalent rotation vector', 2020
global glv
    p2 = []; A2 = []; p3 = []; A3 = [];  p4 = []; A4 = [];
    for m=1:n^3*(n-1)/2
        w = randn(3,n);
        phi1 = polyintn(w);
        phi2 = polyintn(1/2*polycross(phi1,w));
        phi3 = polyintn(polyadd(1/2*polycross(phi2,w), 1/12*polycross(phi1,polycross(phi1,w))));
        phi4 = polyintn(polyadd(1/2*polycross(phi3,w), 1/12*polyadd(polycross(phi1,polycross(phi2,w)),polycross(phi2,polycross(phi1,w)))));
        theta = [];
        for k=1:n
            theta(:,k) = polyvaln(phi1, k/n) - polyvaln(phi1, (k-1)/n);
        end
        % phi2
        p2(3*m-2:3*m,:) = polyvaln(phi2,1);
        ij = []; s=1;
        for i=1:n-1
            for j=i+1:n
                ij(s,:) = [i,j];
                A2(3*m-2:3*m,s) = cross(theta(:,i), theta(:,j)); s=s+1;
            end
        end
        % phi3
        p3(3*m-2:3*m,:) = polyvaln(phi3,1);
        ijk = []; s=1;
        for i=1:n
            for j=1:n-1
                for k=j+1:n
                    ijk(s,:) = [i, j, k];
                    A3(3*m-2:3*m,s) = cross(theta(:,i), cross(theta(:,j), theta(:,k))); s=s+1;
                end
            end
        end
        % phi4
        p4(3*m-2:3*m,:) = polyvaln(phi4,1);
        ijkl = []; s=1;
        for i=1:n
            for j=1:n
                for k=1:n-1
                    for l=k+1:n
                        ijkl(s,:) = [i, j, k, l];
                        A4(3*m-2:3*m,s) = cross(theta(:,i), cross(theta(:,j), cross(theta(:,k), theta(:,l)))); s=s+1;
                    end
                end
            end
        end
    end
    % phi2
    [R,jb] = rref(A2); ij = ij(jb,:);
    K2 = lscov(A2(:,jb),p2);
    % phi3
    [R,jb] = rref(A3); ijk = ijk(jb,:);
    K3 = lscov(A3(:,jb),p3);
    % phi4
    [R,jb] = rref(A4); ijkl = ijkl(jb,:);
    K4 = lscov(A4(:,jb),p4); 
    hoc.N = n;
    hoc.ij = ij; hoc.K2 = K2;
    hoc.ijk = ijk; hoc.K3 = K3;
    hoc.ijkl = ijkl; hoc.K4 = K4;
%     save([glv.rootpath,'\base\base0\highordercoef.mat'], 'hocoef');

