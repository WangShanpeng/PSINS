function psinslog(varname, data, note)
% Save data to log file 'psinslog.txt'.
%
% Prototype: psinslog(varname, data, note)
% Inputs: varname - variable name
%         data - row vector array to save
%         note - some notation
% Usages: 
%    Save: psinslog()  % log date
%          psinslog(data)  % log data only
%          psinslog(varname, data)  % log varname & data
%
% See also  binfile.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/07/2023
global glv
    fid = fopen([glv.datapath, 'psinslog.txt'], 'a');
    if nargin==0
        fprintf(fid, ['\n---', datestr(now,'yyyy-mm-dd HH:MM:SS'), '----------------------\n']);
    elseif nargin==1
        data = varname;
        fprintf(fid, '%f ', data);
        fprintf(fid, '\n');
    elseif nargin>=2
        if nargin==3
            fprintf(fid, '%s = [  %% %s\n', varname, note);
        elseif nargin==2
            if ~ischar(varname)  % psinslog(data, note);
                note = data; data = varname;
                fprintf(fid, '%f ', data);
                fprintf(fid, '%% %s ', note);
                fprintf(fid, '\n');
                fclose(fid);
                return;
            end
            fprintf(fid, '%s = [\n', varname);
        end
        fprintf(fid, '\t');
        fprintf(fid, '%f ', data);
        fprintf(fid, '\n];\n');
    end
    fclose(fid);
