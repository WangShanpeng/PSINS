function data = txtmatfile(fname)
% Convert '.txt' or '.dat' to '.mat' file.
%
% Prototype: data = matbinfile(fname, varname, clm)
% Inputs: fname - file name, with extension '.txt' or '.dat'
% Output: data - data array read from the '.txt' file

% See also  txtbinfile, binmatfile, txt2matfile.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/05/2022
    if isempty(strfind(fname,'*'))
        data = load(fname);
        save([fname(1:end-3),'mat'], 'data');
    else
        [strnames, n] = dirstr(fname);
        for k=1:n
            disp(sprintf('%s  --- is in processing ...\n', strnames{k}));
            txtmatfile(strnames{k});
        end
    end