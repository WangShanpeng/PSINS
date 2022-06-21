function data = matbinfile(fname, varname, clm)
% Convert '.mat' to '.bin' file, or '.bin' to '.mat' file.
%
% Prototype: data = matbinfile(fname, varname, clm)
% Inputs: fname - file name, with extension '.mat' or '.bin'
%         varname - variable name (will be) in mat file
%         clm - column for '.bin' file.
% Output: data - data array read from the '.mat' or '.bin' file

% See also  binfile, txtbinfile, txtmatfile, txt2matfile, fig2jpg.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/05/2020
    if ~isempty(strfind(fname, '.bin'))  % matbinfile('xxx.bin', data, 7)
        if nargin<3, clm = varname; varname = 'data'; end
        data = binfile(fname, clm);
        S = [];
        setfield(S, varname, data);
        save([fname(end-3:end),'.mat'], '-struct', varname);
    else                                % matbinfile('xxx.mat', data)
        if nargin<2, varname=[]; end
        if isempty(varname)
            S = load(fname, '-mat');
            names = fieldnames(S);
            if length(names)>1, error('too many variables in .mat file!'); end
            varname = names{1};
        else
            S = load(fname, '-mat', varname);
        end
        data = getfield(S, varname);
        binfile([fname(1:end-4),'.bin'], data);
    end

