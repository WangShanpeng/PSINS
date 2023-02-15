function R = mycholupdate(R, X, sgn)
% 乔莱斯基分解秩-1更新，R'*R:=R'*R+sgn*X*X',R上三角,X列向量(可多列)
    if nargin<3, sgn=1; end
    [n, m] = size(X);
    if m>1  % 若X为多列，则逐列更新
        for k=1:m, R = mycholupdate(R, X(:,k), sgn); end
        return;
    end
    X = X(:)';  % 转为行向量
    for k=1:n
        s11 = sqrt(R(k,k)^2+sgn*X(k)^2);  % 须非负
        c = R(k,k)/s11;  s = X(k)/s11;
        s12 = c*R(k,k+1:n) + sgn*s*X(k+1:n);
        X(k+1:n) = c*X(k+1:n) - s*R(k,k+1:n);
        R(k,k:n) = [s11,s12];
    end