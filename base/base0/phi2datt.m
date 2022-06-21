function datt = phi2datt(phi, att)
% Translate platform misalignment angles(should be small) to attitude error angles.
% This function is suitable for batch processing.
%
% Prototype: phi = aa2phi(att1, att0)
% Inputs: phi - platform misalignment angles from computed nav-frame to
%               ideal nav-frame, phi=[phiE;phiN;phiU]
%         att - reference Euler angles
% Output: datt - attitude error angles [dpitch; droll; dyaw]
%
% See also  aa2phi, datt2mu.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/06/2022
    [m, n] = size(att);
    if n==1, att=att'; phi=phi'; end
    sp = sin(att(:,1));  cp = cos(att(:,1));
    sy = sin(att(:,3));  cy = cos(att(:,3));
    datt = phi;
    datt(:,1) = cp.*cy.*phi(:,1)+cp.*sy.*phi(:,2);
    datt(:,2) = -sy.*phi(:,1)+cy.*phi(:,2);
    datt(:,3) = sp.*sy.*phi(:,1)+sp.*cy.*phi(:,2)+cp.*phi(:,3);
    datt(:,1:3) = -[datt(:,1)./cp, datt(:,2)./cp, datt(:,3)./cp];
    if n==1, datt=datt'; end
    