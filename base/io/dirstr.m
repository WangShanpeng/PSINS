function [strnames, n] = dirstr(str)
% see also dirfile.
    if nargin<1, str = '*.*'; end
    f = dir(str);
    n = length(f);
    for k=1:n
        strnames{k} = f(k).name;
    end