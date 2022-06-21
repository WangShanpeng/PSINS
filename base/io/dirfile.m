function [fname, m] = dirfile(spc, k)
% Find files in current directory.
%
% Prototype: trj = trjfile(fname, trj)
% Inputs: spc - special file name
%         k - index k
% Output: fname - file names
%         m - number of files
% Example:
%    dirfile('*.txt');
%
% See also  dirstr, rmpathx.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/05/2022
    d = dir(spc);
    for m=1:length(d)
        fname{m,1} = d(m).name;
    end
    if nargin>1; fanme = fname{k}; m=1; end
