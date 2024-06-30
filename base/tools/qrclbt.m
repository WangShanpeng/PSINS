function [Kg, eb, Ka, db, q] = qrclbt(Kg, eb, Ka, db, lu)
% Make SIMU scale matirx Ka to be lower triangular matrix.
%
% Prototype: [Kg, eb, Ka, db] = qrclbt(Kg, eb, Ka, db)
% Inputs: Kg, eb, Ka, db - calibration parameters.
%         lu - lower or upper matrix for Ka
% Outputs: Kg, eb, Ka, db - calibration results with Ka to be lower triangular matrix
%          q - orthogonal-rotation matrix
%
% See also  lsclbt.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/04/2024
    if nargin<=2      %  [clbt, q] = qrclbt(clbt, lu)
        if nargin<2, lu = 'L'; end
        clbt = Kg;
        [clbt.Kg, clbt.eb, clbt.Ka, clbt.db, q] = qrclbt(clbt.Kg, clbt.eb, clbt.Ka, clbt.db, lu);
        Kg = clbt.Kg; eb = q;
        return;
    end
    if nargin<5, lu = 'L'; end
    lu = upper(lu);
    [q, r] = qrlu(Ka, lu);  q = q';
	Ka = r;      eb = q*eb;
	Kg = q*Kg;   db = q*db;
%     if lu=='U'
%         [q, r] = qrp(Ka);  q = q';
%         Ka = r;      eb = q*eb;
%         Kg = q*Kg;   db = q*db;
%     else
%         [q, r] = qrp(Ka');    % lower tri mat, r'*q'=Ka => r'=Ka*q
%         Ka = r';       eb = q'*eb;
%         Kg = Kg*q;     db = q'*db;
%     end
    
function [Q, R] = qrlu(A, lu)
    n = length(A);
    R = zeros(n);
    if lu=='U'
        for i=1:n
            R(i,i) = sqrt(A(:,i)'*A(:,i));
            A(:,i) = A(:,i)/R(i,i);
            j = i+1:n;
            R(i,j) = A(:,i)'*A(:,j);
            A(:,j) = A(:,j)-A(:,i)*R(i,j);
        end
    elseif lu=='L'
        for i=n:-1:1
            R(i,i) = sqrt(A(:,i)'*A(:,i));
            A(:,i) = A(:,i)/R(i,i);
            j = 1:i-1;
            R(i,j) = A(:,i)'*A(:,j);
            A(:,j) = A(:,j)-A(:,i)*R(i,j);
        end
    end
	Q = A;

