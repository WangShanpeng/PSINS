function [fname, m] = dirfile(spc, k)
% Find files in current directory.
%
% Prototype: [fname, m] = dirfile(spc, k)
% Inputs: spc - special file name
%         k - just return one file name index by k, or the output file name
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
    if nargin>1; 
        if ischar(k)  % dirfile(spc, outputfile)
            outputfile = k;
            fid = fopen(outputfile, 'wt');
            for k=1:m
                fprintf(fid, '%s \n', fname{k});
            end
            fclose(fid);
        else
            fname = fname{k}; m=length(k);
        end
    end
