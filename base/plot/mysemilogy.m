function mysemilogy(t, y)
% semilogy plot including negative values, which will been shown in red color.
    if nargin<2; y=t; t=(1:length(y))'; end
    myfigure, 
    for k=1:size(y,2)
        yk = y(:,k);
        idx = abs(yk)>0;
        semilogy(t(idx), abs(yk(idx))); hold on
        idx = yk<0;
        semilogy(t(idx), -yk(idx), '*r');
    end
    xygo;