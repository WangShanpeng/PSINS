function [idx, err] = checkfrmfile(binfile, header, frmlen, isfig)
% Check bin-file-frame correctness, if any wrong, write to new bin-file.
%
% Prototype: [idx, err] = checkfrmfile(binfile, header, frmlen, isfig)
% Input: binfile - input bin file name
%        header - frame header, i.e. 'AA55' ect.
%        frmlen - frame length, include header
% Outputs: idx - correct header index
%          err - error header index
%
% See also  binfrmfile, byte2num, hexbinfile, binfile, matbinfile.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/07/2024 
    fid = fopen(binfile, 'r');
    dd = fread(fid, inf, 'uint8=>uint8');
    fclose(fid);
    dd1 = dd;
    idx = find(dd(1:end-1)==hex2dec(header(1:2)) & dd(2:end)==hex2dec(header(3:4)));  % ttest(idx)
    k_1 = 1;  k1=1;
    for k=2:length(idx)
        if idx(k)-idx(k_1)<frmlen
            idx(k)=0;
        else
            k2 = k1+frmlen;
            dd1(k1:k2-1) = dd(idx(k_1):idx(k_1)+frmlen-1);
            k1=k2;
            k_1=k;
        end
    end
    err = find(idx==0);
    idx(idx==0) = [];
    if ~isempty(err)
        if nargin<4, isfig=1; end
        if isfig==1, myfig,plot(err,'-o'); end
        fid = fopen([binfile,'.frm'], 'w');
        fwrite(fid, dd1, 'uint8');
        fclose(fid);
    end
    