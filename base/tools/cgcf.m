function cgcf(n, dly)
% close the last n current figures
    if nargin<2, dly=1; end
    if nargin<1, n=1; end
    for k=1:n,
        pause(dly);
        close(gcf);
    end

