function pdemo(n, n1)
% PSINS-Demo list.
%
% Prototype: pdemo(n, n1)
% Inputs: n, n1 - n or filenames wildcards
%
% Examples
%    pdemo;          % display all demo files
%    pdemo(10);      % open the 10th demo file
%    pdemo('ekf');   % display demo files with wildcards '*ekf*.m'
%    pdemo('ekf',3); % open the 3rd 'ekf'-demo file
%
% See also pfind.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/01/2024
global glv
    if nargin<2, n1=0; end
    if nargin<1, n=0; end
    if ischar(n), name=['\demos\*',n,'*.m']; n=n1; else, name='\demos\*.m'; end
    [fnames,m] = dirfile([glv.rootpath,name]);
    if n==0
        for k=1:m
            fnames{k}=sprintf('%3d  %s',k,fnames{k});
        end
        disp(fnames);
    else
        open(fnames{n});
    end
%     fid = fopen([glv.rootpath,'\demos\psinsdemo.m'],'w');
%     for k=1:m
%         fprintf(fid, '%s\r\n', fnames{k});
%     end
%     fclose(fid);