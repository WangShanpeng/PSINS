function box3c(qnb, xyz)
% Attitude rotation demo.
%
% Example
%    box3c([90;90;0]*glv.deg);
%    
%    box3c();  q1 = a2qua([90;0;0]*glv.deg); q2 = a2qua([0;90;0]*glv.deg);
%    box3c(q1); box3c(qmul(q1,q2));
%    box3c(q2); box3c(qmul(q2,q1));
%
% See also  att3ddemo.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2024

%              ^ z
%              |
%              |
%      x<----- 0
%               \
%                v y
    if nargin<1; qnb=[1;0;0;0]; end
    if length(qnb)==3, qnb = a2qua(qnb); end
    if nargin<2; xyz=[2;3;1]; end
    x=xyz(1); y=xyz(2); z=xyz(3);
    flu = [ x;  y;  z];  % forward/back,left/right,up/down
    fru = [-x;  y;  z];
	frd = [-x;  y; -z];
    fld = [ x;  y; -z];
    blu = [ x; -y;  z];
    bru = [-x; -y;  z];
	brd = [-x; -y; -z];
    bld = [ x; -y; -z];
    flu = qmulv(qnb,flu); fru = qmulv(qnb,fru); frd = qmulv(qnb,frd); fld = qmulv(qnb,fld);
    blu = qmulv(qnb,blu); bru = qmulv(qnb,bru); brd = qmulv(qnb,brd); bld = qmulv(qnb,bld);
    hold off;  
    fill3([flu(1);fru(1);frd(1);fld(1)], [flu(2);fru(2);frd(2);fld(2)], [flu(3);fru(3);frd(3);fld(3)], 'r');  hold on; % forward
    fill3([blu(1);bru(1);brd(1);bld(1)], [blu(2);bru(2);brd(2);bld(2)], [blu(3);bru(3);brd(3);bld(3)], 'm'); % back
    fill3([flu(1);fru(1);bru(1);blu(1)], [flu(2);fru(2);bru(2);blu(2)], [flu(3);fru(3);bru(3);blu(3)], 'c'); hold on; % up
    fill3([fld(1);frd(1);brd(1);bld(1)], [fld(2);frd(2);brd(2);bld(2)], [fld(3);frd(3);brd(3);bld(3)], 'y'); % down
    fill3([flu(1);blu(1);bld(1);fld(1)], [flu(2);blu(2);bld(2);fld(2)], [flu(3);blu(3);bld(3);fld(3)], 'b'); % right
    fill3([fru(1);bru(1);brd(1);frd(1)], [fru(2);bru(2);brd(2);frd(2)], [fru(3);bru(3);brd(3);frd(3)], 'g'); % left
%     F = qmulv(qnb,[0;y;0]);  B = -F;   text(F(1),F(2),F(3),'F'); text(B(1),B(2),B(3),'B');
%     R = qmulv(qnb,[x;0;0]);  L = -R;   text(L(1),L(2),L(3),'L'); text(R(1),R(2),R(3),'R');
%     U = qmulv(qnb,[0;0;z]);  D = -U;   text(U(1),U(2),U(3),'U'); text(D(1),D(2),D(3),'D');
    m = max(xyz)*2;
    F = qmulv(qnb,[0;m;0])*1.0;
    R = qmulv(qnb,[m;0;0])*1.0;
    U = qmulv(qnb,[0;0;m])*1.0;
    quiver3(0,0,0, F(1),F(2),F(3), 'k','filled','LineWidth',2); text(F(1),F(2),F(3),'Y');
    quiver3(0,0,0, R(1),R(2),R(3), 'k','filled','LineWidth',2); text(R(1),R(2),R(3),'X');
    quiver3(0,0,0, U(1),U(2),U(3), 'k','filled','LineWidth',2); text(U(1),U(2),U(3),'Z');
%     fill3([-x;-1.5*x;0;1.5*x;x;x;-x], [0.6*y;0.6*y;1.3*y;0.6*y;0.6*y;-y;-y], repmat(-m,7,1), 'k'); % bottom;
%     text(0,1.4*y,-m,'North');
    view(55,30); xylim(m,3); xylabel('X','Y','Z'); % grid on;
    