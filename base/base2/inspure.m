function avp = inspure(imu, avp0, href, isfig)
% Process SINS pure inertial navigation with SIMU log data and
% using initial condition avp0 = [att0,vn0,pos0].
%
% Prototype: avp = inspure(imu, avp0, href, isfig)
% Inputs: imu - SIMU data array
%         avp0 - initial parameters, avp0 = [att0,vn0,pos0]
%         href - reference height for altitude damping.
%                If href is a char, then 
%                   'v' - velocity fix-damping
%                   'V' - vertical velocity fix-damping
%                   'p' - position fix-damping
%                   'H' - height fix-damping.
%                   'f' - height free.
%                   'O' - open loop, vn=0.
%                If href is a vector length(href)==length(imu)
%                   'z' - fix vU & hgt, vU=vUref, hgt=href.
%                   'Z' - fix hgt, hgt=href.
% Output: avp - navigation results, avp = [att,vn,pos,t]
%
% See also  insinstant, attpure, trjsimu, insupdate, drpure, nhcpure.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/01/2013, 04/09/2014
global glv
    [nn, ts, nts] = nnts(2, imu(2,end)-imu(1,end));
    if length(avp0)<9, avp0=[avp0(1:3);zeros(3,1);avp0(4:end)]; end  % avp0=[att;pos]
    ins = insinit(avp0, ts);  vn0 = avp0(4:6); pos0 = avp0(7:9);
    if nargin<3,  href = avp0(9);  end
    vp_fix = 'n';
    if length(href)==1
        if ischar(href), vp_fix = href;
        else
            t = imu(10:10:end,end);
            href = [href*ones(size(t)),t];  % all have the same height
        end
        if vp_fix=='O',
            ins.vn0 = zeros(3,1);
            ins.openloop = 1;
        end
    else
        if size(href,2)==3
            vp_fix = 'z';
        elseif size(href,2)==2
            vp_fix = 'Z';
        end
    end
    if vp_fix=='n';
        alt = altfilt(1000, 1*glv.ugpsHz, 10.0, nts);  % altitude damping setting
        imugpssyn(imu(:,end), href(:,2));
        dbU = href; dbU(:,1) = 0;
    end
    len = length(imu);    avp = zeros(fix(len/nn), 10);
    ki = timebar(nn, len, 'Pure inertial navigation processing.');
    for k=1:nn:len-nn+1
        k1 = k+nn-1;
        wvm = imu(k:k1, 1:6);  t = imu(k1,end);
        ins = insupdate(ins, wvm);  % ins.vn(3) = 0;
        if vp_fix=='v',      ins.vn = vn0;
        elseif vp_fix=='V',  ins.vn(3) = vn0(3);
        elseif vp_fix=='p',  ins.pos = pos0; ins.vn(3) = vn0(3);
        elseif vp_fix=='H',  ins.pos(3) = pos0(3);
        elseif vp_fix=='N',  N=0; % no damping
        elseif vp_fix=='n',
            alt = altfilt(alt);
            [khref, dt] = imugpssyn(k, k1, 'F');
            if khref>0
                dh = ins.pos(3)-ins.vn(3)*dt - href(khref,1);
                alt = altfilt(alt, dh);
                ins.pos(3) = ins.pos(3) - alt.xk(3);  % vertical vn&pos feedback
                ins.vn(3) = ins.vn(3) - alt.xk(2);    alt.xk(2:3) = 0;
                dbU(khref,1) = alt.xk(1); % just for plot debug
            end
        elseif vp_fix=='f',
            ins.vn(3) = ins.vn(3);
        elseif vp_fix=='z',
            ins.vn(3) = href(k1,1);  ins.pos(3) = href(k1,2);
        elseif vp_fix=='Z',
            ins.pos(3) = href(k1,1);
        elseif vp_fix=='O',
            ins.vn0 = zeros(3,1);
            ins.openloop = 1;
        else
            error('No SINS type matched!');
        end
        avp(ki,:) = [ins.avp; t]';
        ki = timebar;
    end
    if vp_fix=='n'
        figure, plot(dbU(:,2), dbU(:,1)/glv.ug), xygo('dbU');
    end
    if nargin<4, isfig=1; end
    if isfig==1, insplot(avp); end
