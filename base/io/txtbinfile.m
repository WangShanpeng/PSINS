function data = txtbinfile(fname)
% Convert '.txt' or '.dat' to '.bin' file.
%
% Prototype: data = matbinfile(fname, varname, clm)
% Inputs: fname - file name, with extension '.txt' or '.dat'
% Output: data - data array read from the '.txt' file

% See also  matbinfile, txtmatfile, txt2matfile.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/11/2020
    data = load(fname);
    binfile([fname(1:end-4),'.bin'], data);