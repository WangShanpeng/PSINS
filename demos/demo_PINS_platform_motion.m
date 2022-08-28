% PINS platform motion simulation.
% See also  demo_cone_motion, demo_scull_motion.
% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/02/2022
% *** NOTE for keyboard usage: ******************************
%     'leftarrow'  Yaw ++       'rightarrow'   Yaw --
%     'uparrow'    Pitch ++     'downarrow'    Pitch --
%     '/'          Roll ++       '\'           Roll --
%     '='          R^b_z ++      '='           R^b_z --
%     '0'          Yaw = Pitch = Roll = 0;
%     '9'          Yaw = Roll = 0,   Pitch = 90 deg;
% ***********************************************************
function demo_PINS_platform_motion
    global pf_yaw pf_pch pf_rll
    pf_yaw = 0;  pf_pch = 0;  pf_rll = 0;
    figure;
    set(gcf,'WindowKeyPressFcn',@keypressfcn);
    pfplot();
    
function pfplot()
global pf_yaw pf_pch pf_rll
    deg = pi/180; lw = 5;
    Cyaw = rxyz(pf_yaw*deg,'z'); Cpch = rxyz(-pf_pch*deg,'y'); Crll = rxyz(pf_rll*deg,'x');
    hold off;  pie3([1 1 1],{'','',''});  hold on; plot3([0;0],[0;0],[1,-1]*1.3,'c','linewidth',2); % inner frame
    p=[[0;-1;-1], [0;1;-1], [0;1;1], [0;-1;1]]*1.2;  p1=[[0;-1.6;0],[0;-1.2;0]];  % middle frame
    p=Cyaw*p; p1=Cyaw*p1; p2=-p1;
    plot3(p(1,[1:4,1]),p(2,[1:4,1]),p(3,[1:4,1]),'r','linewidth',lw); plot3(p1(1,:),p1(2,:),p1(3,:),'r','linewidth',2); plot3(p2(1,:),p2(2,:),p2(3,:),'r','linewidth',2); 
    p=[[-1;-1;0], [1;-1;0], [1;1;0], [-1;1;0]]*1.5;  p1=[[-2;0;0],[-1.5;0;0]];  % outer frame
    p=Cyaw*Cpch*p; p1=Cyaw*Cpch*p1; p2=-p1;
    plot3(p(1,[1:4,1]),p(2,[1:4,1]),p(3,[1:4,1]),'linewidth',lw); plot3(p1(1,:),p1(2,:),p1(3,:),'linewidth',2); plot3(p2(1,:),p2(2,:),p2(3,:),'linewidth',2); 
    p=[[-1;-1;0], [1;-1;0], [1;1;0], [-1;1;0]]*1.9; p1=[[1.9;-0.5;0], [2.5;0;0;], [1.9;0.5;0]]; p2=[[-2.5;0;0], [-1.9;0;0;]]; % vehicle
    p=Cyaw*Cpch*Crll*p; p1=Cyaw*Cpch*Crll*p1; p2=Cyaw*Cpch*Crll*p2;
    plot3(p(1,[1:4,1]),p(2,[1:4,1]),p(3,[1:4,1]),'-ok','linewidth',10);  plot3(p1(1,:),p1(2,:),p1(3,:),'-ok','linewidth',10);  plot3(p2(1,:),p2(2,:),p2(3,:),'-ok','linewidth',10); 
    a=2.2; xlim([-a;a]);ylim([-a;a]);zlim([-a;a]);
    note = '\leftarrow:Yaw++; \rightarrow:Yaw--; \uparrow:Pitch++; \downarrow:Pitch--; /:Roll++; \\:Roll--';
    pry = sprintf('(Yaw =%4.0f\\circ;  Pitch =%4.0f\\circ;  Roll =%4.0f\\circ)', pf_yaw, pf_pch, pf_rll);
    title({note,pry});

function keypressfcn(h, evt)
global pf_yaw pf_pch pf_rll
    dang = 5;  deg = pi/180;
    if pf_pch>85 || pf_pch<-85, dang=1; end
    switch evt.Key
        case 'leftarrow'
            pf_yaw = pf_yaw + dang;  if pf_yaw>180, pf_yaw=-179; end
        case 'rightarrow'
            pf_yaw = pf_yaw - dang;  if pf_yaw<-179, pf_yaw=180; end
        case 'uparrow'
            pf_pch = pf_pch + dang;  if pf_pch>=90, pf_pch=90; end
        case 'downarrow'
            pf_pch = pf_pch - dang;  if pf_pch<=-90, pf_pch=-90; end
        case 'slash'  % '/'
            pf_rll = pf_rll + dang;  if pf_rll>180, pf_rll=-179; end
        case 'backslash'  % '\'
            pf_rll = pf_rll - dang;  if pf_rll<-179, pf_rll=180; end
        case 'equal'  % '='
            att = round(m2att(a2mat([pf_pch;pf_rll;pf_yaw]*deg)*rxyz(dang*deg,'z'))/deg);
            pf_pch = att(1); pf_rll = att(2); pf_yaw = att(3); 
        case 'hyphen'  % '-'
            att = round(m2att(a2mat([pf_pch;pf_rll;pf_yaw]*deg)*rxyz(-dang*deg,'z'))/deg);
            pf_pch = att(1); pf_rll = att(2); pf_yaw = att(3); 
        case '0'
            pf_yaw = 0;  pf_pch = 0;  pf_rll = 0;
        case '9'
            pf_yaw = 0;  pf_pch = 90;  pf_rll = 0;
    end
    pfplot();
