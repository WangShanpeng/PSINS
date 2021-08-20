function B = adj(A)
%ADJ Matrix adjoint.
% ADJ(A) is the adjoint matrix of square matrix A.
% It is computed using the Cayley-Hamilton Theorem.
% The inverse of A is: INV(A) = ADJ(A)/det(A).
%
% Matrices that are not invertable still have an adjoint.

%written by Paul Godfrey, April, 1998
%pjg@mlb.semi.harris.com

    ce = poly(eig(A));
    cesize = max(size(ce));
    p = [0 ce(1:(cesize-1))];
    s = (-1)^(max(size(A))+1);
    B = s*polyvalm(p,A);     