function hexbinfile(hexfile, binfile)
% Convert hex text file to bin file.
%
% Prototype: hexbinfile(hexfile, binfile)
% Input: hexfile - input hex file name
% Output: binfile - output bin file name
%
% See also  binfrmfile, txtbinfile, binfile, matbinfile, binmatfile.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/06/2024 
    if(nargin<2), binfile=[hexfile(1:end-3),'bin']; end
    fid = fopen(hexfile,'r'); fid1 = fopen(binfile,'w');
    while 1  
        num = fscanf(fid, '%x', 1024*1024);  if length(num)<1, break; end
        fwrite(fid1, num, 'uint8');
    end
    fclose(fid); fclose(fid1);