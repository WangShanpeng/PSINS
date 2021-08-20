function [eph, obs, otype] = rnx210(nFileName, len)
% Read RINEX-format (Ver2.10) nav/obs file.
%
% See also  rnx210o, rnx210n.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/08/2015
    nFileName(end) = 'n';
    eph = rnx210n(nFileName);
    nFileName(end) = 'o';
    if nargin<2, len = 1000; end
    [obs, head] = rnx210o(nFileName, len);
    lnk = obsEphLink(obs, eph);
    otype = head.otype;
    obs = [obs,lnk];
    otype.ei=length(fieldnames(otype))+1;
    otype.dt=length(fieldnames(otype))+1;