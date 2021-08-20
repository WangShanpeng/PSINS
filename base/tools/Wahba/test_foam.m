B=diag([1;1;-1]); 


    detB = det(B);
    adjBp = adj3(B');  adjBp2 = tr3(adjBp*adjBp');
    BBp = B*B';  B2 = tr3(BBp);
    sB2p2adjBp = sqrt(B2+2*sqrt(adjBp2)); sB2n2adjBp = sqrt(B2-2*sqrt(adjBp2));
    lambda = sqrt(3*B2);
    for k = 1:100
        kappa = (lambda^2-B2)/2;
        zeta = kappa*lambda - detB;
		Psi = (lambda^2-B2)^2 - 8*lambda*detB - 4*adjBp2;
        Psi1 = (lambda+sB2p2adjBp)*(lambda-sB2p2adjBp)*(lambda+sB2n2adjBp)*(lambda-sB2n2adjBp)-8*lambda*detB;
        Psi1 = real(Psi1);
		dPsi = 8*zeta;
		dlambda = Psi / dPsi;
        lambda = lambda - dlambda;
% 		if dlambda<1e-15, break; end
    end
    A = ((kappa+B2)*B+lambda*adjBp-BBp*B)/zeta;



detB = det(B);
adjBp = adj3(B');  adjBp2 = tr3(adjBp*adjBp');
BBp = B*B';  B2 = tr3(BBp);
syms x
expand((x^2-B2)^2 - 8*x*detB-4*adjBp2)
x^4 - 6*x^2 + 8*x - 3
roots([1 0 -6 8 -3])
x=1;
x^4 - 6*x^2 + 8*x - 3
expand((x+3)*(x-1)^3)


syms u11 u12 u13 u21 u22 u23 u31 u32 u33 s1 s2 s3 v11 v12 v13 v21 v22 v23 v31 v32 v33
s1=randn(1); s2=randn(1); s3=randn(1); 
( 0*(s1*s2+s2*s3+s3*s1) + 1*(s1^2+s2^2+s3^2) )*diag([s1,s2,s3]) - ...
    0*(s1+s2+s3)*diag([s2*s3 s1*s3 s1*s2]) + ...
    1*diag([s1^3 s2^3 s3^3])
