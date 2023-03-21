function phi = vn2phi(vn, lti, ts)
% Calculating misalign angles from pure SINS velocity error.
%
% Prototype: phi = vn2phi(vn, lti, ts)
% Inputs: vn - pure SINS velocity error, in most case for static base
%         lti - latitude
%         ts  - velocity sampling interval
% Output: phi - misalignment between calculating navigation frame and real
%               navigation frame
%
% Example:
%   ap0 = [[0;0;1]*glv.deg;glv.pos0];
%   imu = imustatic(ap0, 0.1, 300, imuerrset(0, 0, 0.0001, .1));
%   avp = inspure(imu, [q2att(qaddphi(a2qua(ap0(1:3)),[.1;.1;10]*glv.min));glv.pos0], 'f');
%   phi = vn2phi(avp(:,[4:6,end]), glv.pos0);
%
% See also  vn2phiu, aa2phi, vn2att.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/07/2016
global glv
    if nargin<3, ts = diff(vn(1:2,end)); end
    if length(lti)>1, lti=lti(1); end  % lti = pos;
    phi = vn;  phi(:,1:3)=0;
    ki = timebar(1, length(vn)-fix(10/ts), 'vn2phi processing.');
    k0 = fix(10/ts);
    for k=fix(10/ts):length(vn)
        kts = (1:k)'*ts;
        A = [ones(length(kts),1), kts, kts.^2/2, kts.^3/6];
        aEN = invbc(A'*A)*A'*vn(1:k,1:2);
        [phit, phi0] = prls([aEN(2:4,1);aEN(2:4,2)], lti, kts(end));
        phi(k,1:3) = phit';
        ki = timebar;
    end
    phi(1:k0,1:3) = repmat(phi0', k0, 1);
    figure, 
    subplot(211), plot(phi(:,4), phi(:,1:2)/glv.sec); xygo('phiEN');
    subplot(212), plot(phi(:,4), phi(:,3)/glv.min); xygo('phiU');

function [phit, phi0] = prls(aEN, lti, t)
global glv
    sl = sin(lti); cl = cos(lti); tl = sl/cl;
    wN = glv.wie*cl; wU = glv.wie*sl;
    a1E = aEN(1); a2E = aEN(2); a3E = aEN(3); a1N = aEN(4); a2N = aEN(5); a3N = aEN(6);
    
    uE = a2N/glv.g0; uN = -a2E/glv.g0; uU = uN*tl-a3N/(glv.g0*wN);
    phiE0 = a1N/glv.g0; phiN0 = -a1E/glv.g0; phiU0 = phiN0*tl-uE/wN;
    
    phiE = phiE0 + t*uE + t^2/2*(wU*uN-wN*uU);
    phiN = phiN0 + t*uN - t^2/2*wU*uE;
    phiU = phiU0 + t*uU + t^2/2*wU*uE;
    
    phi0 = [phiE0; phiN0; phiU0]; phit = [phiE; phiN; phiU];