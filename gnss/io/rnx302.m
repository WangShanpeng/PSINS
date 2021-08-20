function [eph, obs, otype] = rnx302(nFileName, len)
% Read RINEX-format (Ver3.02) nav/obs file.
%
% See also  rnx302o, rnx302n, rnx210.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/08/2015
    nFileName(end) = 'n';
    ephn = rnx302n(nFileName);
    nFileName(end) = 'g';
    ephg = rnx302n(nFileName);
    nFileName(end) = 'c';
    ephc = rnx302n(nFileName);
    eph = [ephn;ephg;ephc];
    nFileName(end) = 'o';
    if nargin<2, len = 1000; end
    [obs, head] = rnx302o(nFileName, len);
    [lnk,idx] = obsEphLink(obs, eph);
    otype = head.otype;
    obs = [obs(idx,:),lnk];
    nm = fieldnames(otype); m = 1;
    for k=1:length(nm)
        m = max(m,getfield(otype,nm{k}));
    end
    otype.ei=m+1;
    otype.dt=m+2;