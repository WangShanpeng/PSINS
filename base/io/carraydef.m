function carraydef(valname, array, filename)
% To C/C++ double array define.
%
% Prototype: carraydef(valname, array, filename)
% Inputs: valname - vlaue name
%         array - array
%         filename - write to file
%
% Example
%   carraydef('Kg[]', randn(3));
%   carraydef('Kg[][3]', randn(3));
%
% See also  kfgencpp.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/02/2023
    if nargin<3, fid=1; else, fid = fopen(filename,'wt'); end
    if length(strfind(valname,'['))==2, a2=1; else, a2=0; end
    [m,n] = size(array);
	fprintf(fid, 'double %s = {\n', valname);
    for i=1:m
        for j=1:n
            if a2==1   % 2-dimension array define
                if j==1,    fprintf(fid, '\t{%.8e,', array(i,j));
                elseif j==n,fprintf(fid, '\t%.8e},', array(i,j));
                else        fprintf(fid, '\t%.8e,', array(i,j));
                end
            else,           fprintf(fid, '\t%.8e,', array(i,j));
            end
        end
        fprintf(fid, '\n');
    end        
	fprintf(fid, '};\n');
    if fid~=1, fclose(fid);  end
