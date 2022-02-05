function att1 = adelphi(att, phi)
% Get the accurate attitude from calculated attitude and misalignment
% angles. It can be denoted as 'att1 = att - phi', where att is calculated 
% attitude and phi is misalignment angles.
%
% Prototype: att1 = adelphi(att, phi)
% Inputs: att - attitude input
%         phi - platform misalignment angles from computed nav-frame to
%               ideal nav-frame
% Output: att1 - attitude output
%
% See also  qdelphi, qaddphi, qq2phi, qaddafa, qdelafa, qq2afa, aaddmu.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/11/2021
    if size(att,2)==1
        att1 = q2att(qdelphi(a2qua(att),phi));
    else  % batch processing
        att1 = att;
        for k=1:size(att,1)
            att1(k,1:3) = q2att(qdelphi(a2qua(att(k,1:3)'),phi))';
        end
    end