function err = avpcmpplot(avp0, varargin)
% AVPs comparison & errors plot.
%
% Prototype: err = avpcmpplot(avp0, varargin)
% Inputs: avp0 - AVP reference, may be [att], [att,vn], [att,vn,pos]
%                or [vn,pos]
%         varargin - the last input parameter may be comparison type or by
%                    default value
%          
% See also  avpcmp, posflucmpplot, inserrplot, kfplot, gpsplot, imuplot, avperrstd, errstplot, pos2dplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/10/2013
global glv
    n = size(avp0,2);
    phi_mu = 'phi';
    if ischar(varargin{end})
        if strcmp(varargin{end},'phi')==1, phi_mu = 'phi'; varargin = varargin(1:end-1);
        elseif strcmp(varargin{end},'mu')==1, phi_mu = 'mu'; varargin = varargin(1:end-1);
        elseif strcmp(varargin{end},'datt')==1, phi_mu = 'datt'; varargin = varargin(1:end-1); end
    end
    if ischar(varargin{end}),  ptype = varargin{end};  varargin = varargin(1:end-1);
    else
        if n<3,  	ptype = 'y';
        elseif n<6,	ptype = 'a';
        elseif n<9,	
            if max(abs(avp0(:,4)))>pi/2, ptype = 'av'; else, ptype = 'vp'; end
        else      	ptype = 'avp';
        end
    end
    %%
    myfig;
%     if size(avp0,2)>10, avp0=avp0(:,[1:9,end]); end
    switch ptype
        case {'A', 'a', 'y'},
            avp = avp0; t = avp(:,end); if size(avp,2)<3, avp0=[zeros(length(avp),2),avp]; avp=avp0; end
            subplot(221), plot(t, avp(:,1:2)/glv.deg), xygo('pr');  legend('Pitch Ref.', 'Roll Ref.');
            subplot(223), plot(t, avp(:,3)/glv.deg), xygo('y');
        case 'av',
            avp = avp0; t = avp(:,end);
            subplot(221), plot(t, avp(:,1:3)/glv.deg), xygo('att');  legend('Pitch Ref.', 'Roll Ref.', 'Yaw Ref.');
            subplot(223), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V');
        case 'avp',
            avp = avp0; t = avp(:,end);
            subplot(321), plot(t, avp(:,1:3)/glv.deg), xygo('att');  legend('Pitch Ref.', 'Roll Ref.', 'Yaw Ref.');
            subplot(323), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V');
            subplot(325), plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)]); xygo('DP');
        case 'vp',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
%             subplot(221), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V');
            subplot(221), plot(t, avp(:,4:6)); xygo('V');   legend('V_E Ref.', 'V_N Ref.', 'V_U Ref.');
            subplot(223), plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)]); xygo('DP');
        case 'v',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
%             subplot(121), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V');
            subplot(121), plot(t, avp(:,4:6)); xygo('V');
        case 've',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            subplot(121), plot(t, avp(:,4)); xygo('V');
        case 'vn',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            subplot(121), plot(t, avp(:,5)); xygo('V');
        case 'vu',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            subplot(121), plot(t, avp(:,6)); xygo('V');
        case 'vb',
            avp = avp0; t = avp(:,end);
            vb0 = amulvBatch(avp(:,1:3), avp(:,[4:6,end]));
            subplot(121), plot(t, vb0(:,1:3)); xygo('v^b / m/s');
        case 'p',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            subplot(121), plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)]); xygo('DP');
        case 'ed',
            avp = avp0; t = avp(:,end);
            subplot(211), plot(t, avp(:,end-6:end-4)/glv.dph);
            subplot(212), plot(t, avp(:,end-3:end-1)/glv.ug);
    end
    kk = length(varargin);
    str = '-.--: ';
%     for k=1:kk
%         if size(varargin{k},2)>10, varargin{k}=varargin{k}(:,[1:9,end]); end
%     end
    switch ptype
        case 'A',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
%                subplot(121), hold on, plot(t, avp(:,1:2)/glv.deg, strk, 'LineWidth',2), xygo('pr');
                subplot(221), hold on, plot(t, avp(:,1)/glv.deg, 'r', t, avp(:,2)/glv.deg, 'm'), xygo('pr');
                subplot(222), hold on, plot(t, avp(:,3)/glv.deg, 'r'), xygo('y');
%                err = avpcmp(avp, avp0, 'mu'); t = err(:,end);
%                 subplot(122), hold on, plot(t, err(:,1:2)/glv.min, strk, 'LineWidth',2); xygo('mu'); mylegend('mux','muy');
            end
        case {'a', 'y'},
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);  if size(avp,2)<3, avp=[zeros(length(avp),2),avp]; end
                subplot(221), hold on, plot(t, avp(:,1:2)/glv.deg, strk, 'LineWidth',2), xygo('pr');
                subplot(223), hold on, plot(t, avp(:,3)/glv.deg, strk, 'LineWidth',2), xygo('y');
                err = avpcmp(avp(:,[1:3,end]), avp0(:,[1:3,end]), phi_mu); t = err(:,end);
                subplot(222), hold on, plot(t, err(:,1:2)/glv.min, strk, 'LineWidth',2); xygo(phi_mu);   if phi_mu(1)=='p', mylegend('phiE','phiN'); elseif phi_mu(1)=='m', mylegend('mux','muy'); else, mylegend('dpch','drll'); end
                subplot(224), hold on, plot(t, err(:,3)/glv.min, strk, 'LineWidth',2); xygo(phi_mu);   if phi_mu(1)=='p', mylegend('phiU'); elseif phi_mu(1)=='m', mylegend('muz'); else, mylegend('dyaw'); end
            end
         case 'av',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                subplot(221), hold on, plot(t, avp(:,1:3)/glv.deg, strk, 'LineWidth',2), xygo('att');
                subplot(223), hold on, plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)], strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp(:,[1:6,end]), avp0(:,[1:6,end]), phi_mu); t = err(:,end);
                subplot(222), hold on, plot(t, err(:,1:3)/glv.min, strk, 'LineWidth',2); xygo(phi_mu);  if phi_mu(1)=='p', mylegend('phiE','phiN','phiU'); else, mylegend('mux','muy','muz'); end
                subplot(224), hold on, plot(t, err(:,4:6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE','dVN','dVU');
            end
        case 'avp',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                subplot(321), hold on, plot(t, avp(:,1:3)/glv.deg, strk, 'LineWidth',2), xygo('att');
                subplot(323), hold on, plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)], strk, 'LineWidth',2); xygo('V');
                subplot(325), hold on, plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)], strk, 'LineWidth',2); xygo('DP');
                err = avpcmp(avp(:,[1:9,end]), avp0(:,[1:9,end]), phi_mu); t = err(:,end);
                subplot(322), hold on, plot(t, err(:,1:3)/glv.min, strk, 'LineWidth',2); xygo(phi_mu);  if phi_mu(1)=='p', mylegend('phiE','phiN','phiU'); else, mylegend('mux','muy','muz'); end
                subplot(324), hold on, plot(t, err(:,4:6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE','dVN','dVU');
                subplot(326), hold on, plot(t, [[err(:,7),err(:,8)*cos(avp(1,7))]*glv.Re,err(:,9)], strk, 'LineWidth',2); xygo('dP'); mylegend('dlat','dlon','dH');
            end
        case 'avpdist',  % subplot(326), pos_err vs. distance
            avp = varargin{1};
            close(gcf);
            err = avpcmpplot(avp0, avp, phi_mu);
            pos = avp(:,[7:9,end]);
            [RMh, clRNh] = RMRN(pos);
            dpos = [zeros(1,3);diff(pos(:,1:3))];
            dxyz = [dpos(:,2).*clRNh, dpos(:,1).*RMh, dpos(:,3)];
            dist = cumsum(normv(dxyz));
            dist = interp1(pos(:,end), dist, err(:,end));
            t1 = get(gca,'xlim');
            subplot(326), hold off, plot(dist/1000, [[err(:,7),err(:,8)*cos(avp(1,7))]*glv.Re,err(:,9)]); xygo('dist / km', 'dP');
            x1 = (t1(1)-err(1,end))/(err(1,end)-err(end,end))*(dist(1,1)-dist(end,1))+dist(1,1);
            x2 = (t1(end)-err(1,end))/(err(1,end)-err(end,end))*(dist(1,1)-dist(end,1))+dist(1,1);
            xlim([x1,x2]/1000);
        case 'vp',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
%                 subplot(221), hold on, plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)], strk, 'LineWidth',2); xygo('V');
                subplot(221), hold on, plot(t, avp(:,4:6), strk, 'LineWidth',2); xygo('V');
                subplot(223), hold on, plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)], strk, 'LineWidth',2); xygo('DP');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(222), hold on, plot(t, err(:,4:6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE','dVN','dVU');
                subplot(224), hold on, plot(t, [[err(:,7),err(:,8)*cos(avp(1,7))]*glv.Re,err(:,9)], strk, 'LineWidth',2); xygo('dP'); mylegend('dlat','dlon','dH');
            end
        case 'v',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
%                 subplot(121), hold on, plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)], strk, 'LineWidth',2); xygo('V');
                subplot(121), hold on, plot(t, avp(:,4:6), strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,4:6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE','dVN','dVU');
            end
        case 've',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                subplot(121), hold on, plot(t, avp(:,4), strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,4), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE');
            end
        case 'vn',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                subplot(121), hold on, plot(t, avp(:,5), strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,5), strk, 'LineWidth',2); xygo('dV'); mylegend('dVN');
            end
        case 'vu',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                subplot(121), hold on, plot(t, avp(:,6), strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVU');
            end
        case 'vb',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                vb = amulvBatch(avp(:,1:3),avp(:,[4:6,end]));
                subplot(121), hold on, plot(t, vb(:,1:3), strk, 'LineWidth',2); xygo('v^b / m/s');
                err = avpcmp(vb, vb0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,1:3), strk, 'LineWidth',2); xygo('\deltav^b / m/s'); mylegend('dVR','dVF','dVUp');
            end
        case 'p',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                subplot(121), hold on, plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)], strk, 'LineWidth',2); xygo('DP');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, [[err(:,7),err(:,8)*cos(avp(1,7))]*glv.Re,err(:,9)], strk, 'LineWidth',2); xygo('dP'); mylegend('dlat','dlon','dH');
            end
        case 'ed',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                subplot(211), hold on, plot(t, avp(:,end-6:end-4)/glv.dph, strk, 'LineWidth',2); xygo('eb');
                subplot(212), hold on, plot(t, avp(:,end-3:end-1)/glv.ug, strk, 'LineWidth',2); xygo('db');
            end
    end
    
