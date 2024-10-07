function t = cnt2t(cnt, ts, t0)
% Convert wrapped uint8 count [0,255] or uint16 count [0,65535] to 
% continuous time tag.
%
% Prototype: t = cnt2t(cnt, ts)
% Inputs: cnt - wrapped integer count [0,255] or [0,65535
%         ts - sampling interval
%         t0 - start time
% Output: t - continuous time tag
%
% See also  dhms2t.

% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/03/2018
    if ~exist('ts', 'var'), ts = 1; end
    if ~exist('t0', 'var'), t0 = ts; end
    dcnt = diff(cnt);
    m = max(cnt);  idx=dcnt<0;
    if m>10 && m<=15
%         dcnt(dcnt==-15) = 1;
        dcnt(idx) = dcnt(idx)+16;
    elseif m>245 && m<=255
%         dcnt(dcnt==-255) = 1;
        dcnt(idx) = dcnt(idx)+256;
    elseif m>65530 && m<=65535
%         dcnt(dcnt==-65535) = 1;
        dcnt(idx) = dcnt(idx)+65536;
    else
        idx = dcnt<0;
        dcnt(idx) = dcnt(1);
%         dcnt(idx) = dcnt(idx)+m;
    end
    t = cumsum([cnt(1);dcnt])*ts;
	t = t - t(1) + t0;
