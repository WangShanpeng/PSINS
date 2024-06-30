function xylabel(x,y,z)
    xlabel(x); ylabel(y);
    if nargin==3, zlabel(z); end