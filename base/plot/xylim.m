function xylim(x,y,z)
    if length(x)==1, x=[-x,x]; end
    if length(y)==1
        if y==2, xylim(x, x);               % xylim(x,2);
        elseif y==3, xylim(x, x, x); end    % xylim(x,3);
        return;
    end
    xlim(x); ylim(y);
    if nargin==3, zlim(z); end