function funs = pfind(str, tmany)
% PSINS-find functions.
%
% Prototype: funs = pfind(str, tmany)
% Inputs: str - specific string contained
%         tmany - max function to be found
% Outupts: funs - function name list
%
% Examples
%   pfind att;
%   pfind att inf;
%   pfind imu 10;
%
% See also pdemo.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/05/2024
narginchk(1, 2);
dirs = {
    '', ...
    'base', ...
    'base\ahrs', ...
    'base\align', ...
    'base\base0', 'base\base1', 'base\base2'...
    'base\imu', ...
    'base\io', ...
    'base\kf', ...
    'base\plot', ...
    'base\tools', ...
    'base\usedef', ...
    'cns', ...
    'dlg', ...
    'gnss', ...
    'gnss\BeiDou', 'gnss\glonass', 'gnss\gps', 'gnss\io', ...
    'mytest', ...
    };
if nargin && ~ischar(str)
	error(message('PSINS:help:NotAString'));
end
[rpath, ~, ~] = psinsenvi();
funs = {'No file found.'}; n1 = 1;
toomany=20; if nargin>1 toomany=str2num(tmany); end  % findp xxx 50
for k=1:length(dirs)
    stri = [rpath,'\',dirs{k},'\*',str,'*.m'];
    [fname, m] = dirfile(stri);
    for n=1:m,
        fid = fopen(fname{n},'r'); fgetl(fid); tl=fgetl(fid); fclose(fid);      
        funs{n1,:} = [fname{n}(1:end-2), '   ', tl(2:end)];  n1=n1+1;
        if n1>toomany, funs{n1,:}='--- Too many functions found ...'; break; end
    end
    if n1>toomany, break; end
end
if nargout==0, disp(funs); end
