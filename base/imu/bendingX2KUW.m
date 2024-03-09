function [K, U, W] = bendingX2KUW(X, U, W)
% See also  bending, bendingH, bendingplot.
    if nargin==3  % X = bendingX2KUW(K, U, W)
        K = X;
        X = [K(1,1:3)';K(2,2:3)';K(3,3)'; reshape(U',9,1); reshape(W',9,1)];
        K = X;
        return;
    end
    if length(X)==1, X=zeros(24,1); end  % [K, U, W] = bendingX2KUW(X)
    if length(X)==21, X=[0;X(1:2);0;X(3);0;X(4:end)]; end
    K = [X(1:3)';0,X(4:5)';0,0,X(6)];
    U = reshape(X(7:15),3,3)';
    if length(X)>15
        W = reshape(X(16:24),3,3)';
    else
        W = zeros(3);
    end
