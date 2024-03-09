function att = qis2att(qis, utc0, pos, isfig)
% Translate CNS i-frame quaternion to n-frame attitude.
%
% Prototype: att = qis2att(qis, utc0, pos)
% Inputs: qis - CNS quaternion output
%         uct0 - =[year month day seconds]
%         pos - CNS postion [lat lon hgt]
% Output: att - local attitude [pitch, roll, yaw]
%
% See also  cnsCie.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/11/2022
global glv
    if nargin<4, isfig=0; end
    if nargin<3, pos=glv.pos0; end
    att = qis(:,1:4);
    Cie0 = cnsCie(utc0(1:3), utc0(4));
    Cen = pos2cen(pos);
    for k=1:length(qis)   
        Cin = Cie0*rxyz(qis(k,5)*glv.wie,'z')*Cen;
        atti = m2att(Cin'*q2mat(qis(k,:)));  % Cns
        att(k,:) = [ atti; qis(k,5) ]';
    end
    if isfig==1,  insplot(att, 'a');  end
