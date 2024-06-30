function zpinserr(lat)
% Z-plane plot of INS error.
%
% Prototype: zpinserr(lat)
% Input: lat - latitude
%
% See also  etm.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/03/2024
global glv
    if nargin<2; lat=45*glv.deg; end;
    ws = sqrt(glv.g0/glv.Re);  wf = glv.wie*sin(lat);
    z = [glv.wie; ws+wf; ws-wf]/ws;
    zplane([], [z; -z]*1i);  xygo(' ', 'img / \omega_s');