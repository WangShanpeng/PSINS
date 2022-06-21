function rmpathx(pname)
% Remove specific directory from Matlab search path..
%
% Prototype: rmpathx(pname)
% Input: pname - path name contain this tring

% See also  dirstr, dirfile.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/05/2022
    if nargin<1, pname='psinsyymmdd'; end
    if length(pname)<3, return; end  % too less length
    pname = upper(pname);
    pp = upper([';',path,';']);
    ksemicolon = strfind(pp, ';');
    for k=1:length(ksemicolon)-1
        stri = pp(ksemicolon(k)+1:ksemicolon(k+1)-1);
        if strcmp(pname,'~MATLAB')==1  % rmpathx('~matlab') = remove all the 'no-matlab' path
            if isempty(strfind(stri,'MATLAB')), rmpath(stri); end
        else
            if ~isempty(strfind(stri,pname)), rmpath(stri); end
        end
    end