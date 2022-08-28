function seg = trjsegment(seg, segtype, lasting, w, a, var1)
% Add trj-segment setting for trajectory simulator.
%
% Prototype: seg = trjsegment(seg, segtype, lasting, w, a, var1)
% Inputs: seg - trjsegment structure array
%         segtype - trjsegment type
%         lasting - segment lasting time
%         w - trajectory angular rate (NOTE: in deg/sec!)
%         a - trajectory acceleration in m/ss
%         var1 - augmented input, see the following code in detail
% Output: seg - new trjsegment structure array
%          
% See also  trjsimu, insupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/01/2014
    dps = pi/180/1;  % deg/second
    if exist('w', 'var')
        cf = (w*dps)*seg.vel; % centripetal force
    end
    switch(segtype)
        % init ------
        case 'init'          % trjsegment([], 'init', initvelocity)
            initvelocity = lasting;
            seg = [];
            seg.vel = initvelocity;  seg.wat = [];
        % basic ------
        case 'uniform',      % seg = trjsegment(seg, 'uniform', lasting)
            seg.wat = [seg.wat; [lasting, seg.vel, 0, 0, 0, 0, 0, 0]];
        case 'accelerate',   % seg = trjsegment(seg, 'accelerate', lasting, [], a);
            seg.wat = [seg.wat; [lasting, seg.vel, 0, 0, 0, 0, a, 0]];
            seg.vel = seg.vel + lasting*a;
        case 'deaccelerate', % seg = trjsegment(seg, 'deaccelerate', lasting, [], a);  % NOTE: a>0
            seg.wat = [seg.wat; [lasting, seg.vel, 0, 0, 0, 0,-a, 0]];  % a>0
            seg.vel = seg.vel - lasting*a;
        case 'headup',       % seg = trjsegment(seg, 'headup', lasting, w); 
            seg.wat = [seg.wat; [lasting, seg.vel, w*dps, 0, 0, 0, 0, cf]];
        case 'headdown',     % seg = trjsegment(seg, 'headdown', lasting, w);  % NOTE: w>0
            seg.wat = [seg.wat; [lasting, seg.vel,-w*dps, 0, 0, 0, 0,-cf]];
        case 'turnleft',     % seg = trjsegment(seg, 'turnleft', lasting, w);
            seg.wat = [seg.wat; [lasting, seg.vel, 0, 0, w*dps,-cf, 0, 0]];
        case 'turnright',    % seg = trjsegment(seg, 'turnright', lasting, w);  % NOTE: w>0
            seg.wat = [seg.wat; [lasting, seg.vel, 0, 0,-w*dps, cf, 0, 0]];
        case 'rollleft',     % seg = trjsegment(seg, 'rollleft', lasting, w);  % NOTE: w>0
            seg.wat = [seg.wat; [lasting, seg.vel, 0,-w*dps, 0, 0, 0, 0]];
        case 'rollright',    % seg = trjsegment(seg, 'rollright', lasting, w);
            seg.wat = [seg.wat; [lasting, seg.vel, 0, w*dps, 0, 0, 0, 0]];
        % compound ------
        case 'static',       % seg = trjsegment(seg, 'static', lasting, [], [], a);  % NOTE: var1>0
            a = var1;  dacclasting = seg.vel/a;
            seg = trjsegment(seg, 'deaccelerate',  dacclasting, 0, a);
            seg.vel = 0;
            seg = trjsegment(seg, 'uniform',  max(lasting,dacclasting)-dacclasting);
        case 'coturnleft',   % seg = trjsegment(seg, 'coturnleft', lasting, w, [], rolllasting);  % coordinate turn left
            rolllasting = var1; rollw = atan(cf/9.8)/dps/rolllasting;
            seg = trjsegment(seg, 'rollleft',  rolllasting, rollw);
            seg = trjsegment(seg, 'turnleft',  lasting, w);
            seg = trjsegment(seg, 'rollright', rolllasting, rollw);
        case 'coturnright', % seg = trjsegment(seg, 'coturnright', lasting, w, [], rolllasting);  % coordinate turn right
            rolllasting = var1; rollw = atan(cf/9.8)/dps/rolllasting;
            seg = trjsegment(seg, 'rollright', rolllasting, rollw);
            seg = trjsegment(seg, 'turnright', lasting, w);
            seg = trjsegment(seg, 'rollleft',  rolllasting, rollw);
        case '8turn',       % seg = trjsegment(seg, '8turn', [], w, [], rolllasting); 
            lasting = 360/w;
            rolllasting = var1;
            seg = trjsegment(seg, 'coturnleft',  lasting, w, 0, rolllasting);
            seg = trjsegment(seg, 'coturnright', lasting, w, 0, rolllasting);
        case 'sturn',      % seg = trjsegment(seg, 'sturn', [], w, [], rolllasting); 
            lasting1 = 90/w; lasting2 = 180/w;
            rolllasting = var1;
            seg = trjsegment(seg, 'coturnright', lasting1, w, 0, rolllasting);
            seg = trjsegment(seg, 'coturnleft',  lasting2, w, 0, rolllasting);
            seg = trjsegment(seg, 'coturnright', lasting1, w, 0, rolllasting);
            seg = trjsegment(seg, 'coturnleft',  lasting1, w, 0, rolllasting);
            seg = trjsegment(seg, 'coturnright', lasting2, w, 0, rolllasting);
            seg = trjsegment(seg, 'coturnleft',  lasting1, w, 0, rolllasting);
        case 'climb',     % seg = trjsegment(seg, 'climb', lasting, w, [], uniformlasting); 
            uniformlasting = var1;
            seg = trjsegment(seg, 'headup',   lasting, w);
            seg = trjsegment(seg, 'uniform',  uniformlasting);
            seg = trjsegment(seg, 'headdown', lasting, w);
        case 'descent',  % seg = trjsegment(seg, 'descent', lasting, w, [], uniformlasting); 
            uniformlasting = var1;
            seg = trjsegment(seg, 'headdown', lasting, w);
            seg = trjsegment(seg, 'uniform',  uniformlasting);
            seg = trjsegment(seg, 'headup',   lasting, w);
        % user ------
        case 'user',     % seg = trjsegment(seg, 'user', lasting, w, a); 
            seg.wat = [seg.wat; [lasting, seg.vel, w'*dps, a']];
        otherwise,
            error('trjsegment type mismatch.');
    end

