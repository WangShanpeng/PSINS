function insplot(avp, ptype, varargin)
% avp plot.
%
% Prototype: insplot(avp, ptype)
% Inputs: avp - may be [att], [att,vn] or [att,vn,pos]
%         ptype - plot type define
%          
% See also  attplot, miniplot, inserrplot, kfplot, gpsplot, imuplot, magplot, dvlplot, addyawplot, tscaleset.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/10/2013, 09/11/2018
global glv
    t = avp(:,end);
    n = size(avp,2);
    if nargin<2
        if n==2,     ptype = 'y';  % yaw
        elseif n==3, ptype = 'l';  % lat-lon
        elseif n<5,	 ptype = 'a';
        elseif n<8,	 ptype = 'av';
        elseif n<11, ptype = 'avp';
        elseif n==11, ptype = 'avpo';
        elseif n<17, ptype = 'avped';
        elseif n<20, ptype = 'avpedl';
        else         ptype = 'avpedlt';
        end
    end
    %%
    switch ptype
        case 'y',
            myfigure;
            plot(t, avp(:,1)/glv.deg), xygo('y');
        case 'l',
            myfig;
            dxyz = pos2dxyz([avp(:,1:2),avp(:,1)*0]);
            plot(0, 0, 'rp');
            hold on, plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
        case 'A',
            myfig;
            plot(t, avp(:,1:2)/glv.deg), xygo('pr');
        case 'a',
            myfig;
            subplot(211), plot(t, avp(:,1:2)/glv.deg), xygo('pr');
            subplot(212), plot(t, avp(:,3)/glv.deg), xygo('y');
        case 'v',
            myfig;
            plot(t, avp(:,1:3)), xygo('vn');
        case 'p',
            myfig;
            dxyz = pos2dxyz(avp(:,1:3));
            plot(t, dxyz(:,[2,1,3])); xygo('DP'); legend('\DeltaLat','\DeltaLon','\DeltaHgt');
        case 'av',
            myfigure;
            subplot(221), plot(t, avp(:,1:2)/glv.deg); xygo('pr');
            subplot(223), plot(t, avp(:,3)/glv.deg); xygo('y');
            subplot(222), plot(t, avp(:,4:5)); xygo('VEN');
            subplot(224), plot(t, avp(:,6)); xygo('VU');
        case 'vb',
            avp(:,4:6) = amulvBatch(avp(:,1:3),avp(:,4:6));
            if n<8, ptype='av';
            elseif n<11, ptype='avp';
            elseif n<17, ptype='avped'; end
            insplot(avp, ptype);
            if n<8, subplot(222); xygo('v_{x,y}^b / (m/s)'); subplot(224); xygo('v_z^b / (m/s)');
            else, subplot(323); xygo('V ^b / (m/s)'); end
        case 'avp',
            if size(avp,2)==9, t=1:length(t); end
            myfig;
            subplot(321), plot(t, avp(:,1:2)/glv.deg); xygo('pr'); legend('Pitch','Roll');
            subplot(322), plot(t, avp(:,3)/glv.deg); xygo('y'); legend('Yaw');
            subplot(323), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+0*avp(:,6).^2)]); xygo('V'); legend('V_E','V_N', 'V_U', '|V|');
%             subplot(323), plot(t, avp(:,4:6)); xygo('V'); legend('V_E','V_N', 'V_U');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(325), plot(t, dxyz(:,[2,1,3])); xygo('DP'); legend('\DeltaLat','\DeltaLon','\DeltaHgt');
%             subplot(325), plot(t, [[avp(:,7)-avp(1,7),(avp(:,8)-avp(1,8))*cos(avp(1,7))]*glv.Re,avp(:,9)-avp(1,9)]); xygo('DP');
%             subplot(3,2,[4,6]), plot(r2d(avp(:,8)), r2d(avp(:,7))); xygo('lon', 'lat');
%                 hold on, plot(r2d(avp(1,8)), r2d(avp(1,7)), 'rp');
            subplot(3,2,[4,6]), plot(0, 0, 'rp');   % 19/04/2015
                hold on, plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
%                 hold on, plot((avp(:,8)-avp(1,8))*glv.Re*cos(avp(1,7)), (avp(:,7)-avp(1,7))*glv.Re); xygo('est', 'nth');
            legend(sprintf('LON0:%.2f, LAT0:%.2f (DMS), H0:%.1f (m)', r2dms(avp(1,8)),r2dms(avp(1,7)),avp(1,9)));
        case 'avpo',
            insplot(avp(:,[1:9,end]));
            if size(avp,2)~=11, dist = distance(avp(:,[7:9,end])); dist=dist(:,[1,end]); else, dist=avp(:,10:11); end
            subplot(3,2,5), plot(dist(:,2), dist(:,1), 'm');  legend('\DeltaLat','\DeltaLon','\DeltaHgt', 'Dist');  % plot OD distance
        case {'t/m', 't/h', 't/d'},  % AVP-plot where t-axis in day
            tscalepush(ptype);
            insplot([avp(:,1:9),avp(:,end)/tscaleget()],'avp');
            tscalepop();
        case 'ap',
            insplot([avp(:,1:3),zeros(length(avp),3),avp(:,4:end)],'avp');
            subplot(323), delete(gca); %cla;  xygo('N/A'); legend('');
        case 'vp',
            insplot([zeros(length(avp),3),avp],'avp');
        case 'vuh',  % plot VU & hgt
            if n>9, vuh=avp(:,[6,9,end]);   % avp input
            elseif n>6, vuh=avp(:,[3,6,end]); end  % vp input
            myfig;
            ax = plotyy(vuh(:,end), vuh(:,1), vuh(:,end), vuh(:,2));
            xlabel('t / s'); grid on;
            ylabel(ax(1), 'V_U / m/s'); ylabel(ax(2), 'h / m');
        case 'avped'
            myfigure;
            subplot(321), plot(t, avp(:,1:2)/glv.deg); xygo('pr'); legend('Pitch','Roll');
            subplot(322), plot(t, avp(:,3)/glv.deg); xygo('y'); legend('Yaw');
            subplot(323), plot(t, [avp(:,4:6),normv(avp(:,4:6))]); xygo('V'); legend('V_E','V_N', 'V_U', '|V|');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(324), plot(t, dxyz(:,[2,1,3])); xygo('DP'); legend('\DeltaLat','\DeltaLon','\DeltaHgt');
%             subplot(324), plot(t, [[avp(:,7)-avp(1,7),(avp(:,8)-avp(1,8))*cos(avp(1,7))]*glv.Re,avp(:,9)-avp(1,9)]); xygo('DP');
            subplot(325), plot(t, avp(:,10:12)/glv.dph); xygo('eb'); legend('\epsilon_x','\epsilon_y','\epsilon_z');
            subplot(326), plot(t, avp(:,13:15)/glv.ug); xygo('db'); legend('\nabla_x','\nabla_y','\nabla_z');
        case 'avpedl'
            myfigure;
            subplot(421), plot(t, avp(:,1:2)/glv.deg); xygo('pr');
            subplot(422), plot(t, avp(:,3)/glv.deg); xygo('y');
            subplot(423), plot(t, [avp(:,4:6),normv(avp(:,4:6))]); xygo('V');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(424), plot(t, dxyz(:,[2,1,3])); xygo('DP');
            subplot(425), plot(t, avp(:,10:12)/glv.dph); xygo('eb');
            subplot(426), plot(t, avp(:,13:15)/glv.ug); xygo('db');
            subplot(427), plot(t, avp(:,16:18)); xygo('L');
        case 'avpedlt'
            insplot(avp, 'avpedl');
            subplot(428), plot(t, avp(:,19)); xygo('dT');
        case 'trjpy'  
            py = vn2att(avp(:,[4:6,end]));
            p1 = interp1(avp(:,end), avp(:,1), py(:,end));
            y1 = interp1(avp(:,end), avp(:,3), py(:,end));
            myfigure;
            subplot(321), plot(t, avp(:,1:2)/glv.deg, py(:,end), py(:,1)/glv.deg); xygo('pr'); legend('pitch','roll','pitch_{trj}');
            subplot(322), plot(t, avp(:,3)/glv.deg, py(:,end), py(:,3)/glv.deg); xygo('y'); legend('yaw','yaw_{trj}');
            subplot(323), plot(py(:,end), [p1-py(:,1)]/glv.deg); xygo('\delta\theta / ( \circ )'); legend('pitch_{trj}-pitch'); % attack angle
            subplot(324), plot(py(:,end), asin(sin(y1-py(:,3)))/glv.deg); xygo('\delta\psi / ( \circ )'); legend('yaw_{trj}-yaw'); % drift angle
            subplot(325), plot(t, [normv(avp(:,4:5)),avp(:,6)]); xygo('V'); legend('|V_{EN}|', 'V_U');
            subplot(326), plot(t, avp(:,4:5)); xygo('V'); legend('V_E', 'V_N');
        case 'DR',
            avptrue = setvals(varargin);
            myfigure;
            subplot(321), plot(t, avp(:,1:2)/glv.deg); xygo('pr');
            subplot(322), plot(t, avp(:,3)/glv.deg); xygo('y');
            subplot(323), plot(t, [avp(:,4:6),normv(avp(:,4:6))]); xygo('V');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(325), plot(t, dxyz(:,[2,1,3])); xygo('DP');
%             subplot(325), plot(t, [[avp(:,7)-avp(1,7),(avp(:,8)-avp(1,8))*cos(avp(1,7))]*glv.Re,avp(:,9)-avp(1,9)]); xygo('DP');
            subplot(3,2,[4,6]), plot(r2d(avp(:,8)), r2d(avp(:,7))); xygo('lon', 'lat');
                hold on, plot(r2d(avptrue(:,8)), r2d(avptrue(:,7)), 'r-');
                plot(r2d(avp(1,8)), r2d(avp(1,7)), 'rp');
            legend('DR trajectory', 'True trajectory', 'Location','Best');
    end
        