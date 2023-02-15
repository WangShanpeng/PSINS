function fname1 = fnameext(fname, ext)
% Modify filename extension
%
% Prototype: fname1 = fileext(fname, ext)
% Inputs: fname - old filename
%         ext - extension
% Outputs: fname - new filename
% 
% Example:
%     fname1 = fileext('data.bin', 'txt');
% 
% See also  fnamechk.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/02/2023
    idx = strfind(fname, '.');
    if isempty(idx), idx=length(fname)+1; end
    fname1 = [fname(1:idx(end)-1),'.',ext];