function att = attrf(att, dir)
% Attitude transform from rocket-board to fligh-board, or vice versa.
%
% Prototype: att = attrf(att, dir)
% Inputs: att - input Euler angle array, must be in R/x-F/y-U/z frame
%         dir - flag 0 from rocket-board to fligh-board, 1 from fligh-board to rocket-board
% Output: att - output Euler angle
%
% See also  a2mat, a2qua1, axxx2a.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/03/2022
    if nargin<2, dir=0; end
    batch = 1;
    if size(att,2)==1, att=att'; batch=0; end
    s = sin(att); c = cos(att);
    si = s(:,1); sj = s(:,2); sk = s(:,3); 
    ci = c(:,1); cj = c(:,2); ck = c(:,3);
    if dir==0  % rocket to flight
        C12 = -sk;
        C22 = ci.*ck;
        C31 = cj.*si.*sk - ci.*sj; C32 = ck.*si; C33 = ci.*cj + si.*sj.*sk;
        att(:,1:3) = [ asin(C32), atan2(-C31,C33), atan2(-C12,C22) ];
    else  % flight to rocket
        C11 = cj.*ck-si.*sj.*sk; C12 = -ci.*sk;  C13 = sj.*ck+si.*cj.*sk;
        C22 = ci.*ck;
        C32 = si;
        att(:,1:3) = [ atan2(C32,C22), atan2(C13,C11), -asin(C12) ];
    end
    if batch==0, att=att'; end
    
%     syms si ci sj cj sk ck
%     Cp = [1 0 0; 0 ci -si; 0 si ci];
%     Cr = [cj 0 sj; 0 1 0; -sj 0 cj];
%     Cy = [ck -sk 0; sk ck 0; 0 0 1];
%     C = Cp*Cy*Cr 
% [            cj*ck,   -sk,            ck*sj]
% [ si*sj + ci*cj*sk, ci*ck, ci*sj*sk - cj*si]
% [ cj*si*sk - ci*sj, ck*si, ci*cj + si*sj*sk]