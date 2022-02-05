function avarfit(sigma, tau)
% Gyro Allan variance fit tool.
%
% Prototype: avarfit(sigma, tau)
% Inputs: sigma - gyro Allan variance array, in deg/hur
%         tau - Allan variance correlated time array, in sec
%
% Example: 
%     glvs;
%     y = avarsimu([0.001,0.05,0.01,.0], [1, 1], 0.01, 1000000, 1);
%     [sigma,tau] = avar(y, 0.01);  avarfit(sigma, tau);
%
% See also  avar, avarsimu, lmc.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/05/2021
global gtau hls ys mkva1 mkvb1 mkva2 mkvb2 htext
    if length(sigma)>1000  %  avarfit(gyro_data_in_dph, ts);
        [sigma, tau] = avar(sigma, tau);
        avarfit(sigma, tau);
        return;
    end
    if length(sigma)<10, error('The length of input sigma is too short.'); end
    if nargin<2,
        tau=0.01;
        if size(sigma,2)==2, tau=sigma(:,end); sigma=sigma(:,1); end
    end
    if length(tau)==1, tau=tau*2.^(0:length(sigma)-1)'; end  % ts*[1,2,4,8,...]';
    gtau = tau;
    ys = [tau.^-1, tau.^-0.5, tau.^0, tau.^0.5, tau.^1, ararmkv(1,1), ararmkv(1,1)];
    ys(:,1) = ys(:,1)*(sigma(1)/ys(1,1));
    ys(:,2) = ys(:,2)*(sigma(4)/ys(4,2));
    [m,idx] = min(sigma); m=m/100000; mkva1=m; mkvb1=m*100; mkva2=m/2; mkvb2=m*10;
    ys(:,3) = ys(:,3)*(sigma(idx)/ys(idx,3));
    ys(:,4) = ys(:,4)*(sigma(end-4)/ys(end-4,4));
    ys(:,5) = ys(:,5)*(sigma(end)/ys(end,5));
    ys(:,6) = ararmkv(mkva1,mkvb1);
    ys(:,7) = ararmkv(mkva2,mkvb2);
    ys(:,8) = normv(ys(:,1:7),2);
    myfig; hls(8)=loglog(tau, ys(:,8), '-g', 'linewidth',2); xygo('\it\tau\rm / s', '\it\sigma\rm_A / ( ( \circ ) / h )');
    hls(9)=loglog(tau, sigma, '-m*', 'linewidth',2);  legend('AVAR fitted', 'AVAR original');
    str = ['Q '; 'N '; 'B '; 'K '; 'R '; 'M1'; 'M2'];
    for k=1:7
        hls(k) = plot(tau, ys(:,k), '--');
        set(hls(k), 'ButtonDownFcn', {@bdfcn,k});
        htext(k) = text(tau(end)*1.5, ys(end,k), str(k,:));
    end
    dispQNBKR();

    function bdfcn(~,~,hk)
        global prep hls
        set(hls(hk),'LineWidth',2,'Selected','on');
        prep = get(gca,'CurrentPoint');
        set(gcf,'WindowButtonMotionFcn',{@wbmfcn,hk});
        set(gcf,'WindowButtonUpFcn',{@wbufcn,hk});

    function wbmfcn(~,~,hk)
        global gtau prep hls ys mkva1 mkvb1 mkva2 mkvb2
        curp = get(gca,'CurrentPoint');
        if hk<6
            ys(:,hk) = ys(:,hk)*(curp(1,2)/prep(1,2));
        elseif hk==6
            [~,idx] = max(ys(:,6));
            if idx>2 && curp(1,1)<gtau(idx)
                mkva1 = mkva1*curp(1,2)/prep(1,2);
            elseif idx<length(gtau)-1 && gtau(idx)<curp(1,1)
                mkvb1 = mkvb1*curp(1,2)/prep(1,2);
            end
            ys(:,6) = ararmkv(mkva1, mkvb1);
        elseif hk==7
            [~,idx] = max(ys(:,7));
            if idx>2 && curp(1,1)<gtau(idx)
                mkva2 = mkva2*curp(1,2)/prep(1,2);
            elseif idx<length(gtau)-1 && gtau(idx)<curp(1,1)
                mkvb2 = mkvb2*curp(1,2)/prep(1,2);
            end
            ys(:,7) = ararmkv(mkva2, mkvb2);
        end
        prep = curp;
        ys(:,8) = normv(ys(:,1:7),2);
        set(hls(hk),'YData',ys(:,hk));
        set(hls(8),'YData',ys(:,8));
        dispQNBKR();

    function wbufcn(~,~,hk)
        global hls
        set(gcf,'WindowButtonMotionFcn','');
        set(gcf,'WindowButtonUpFcn','');
        set(hls(hk),'LineWidth',1,'Selected','off');

    function dispQNBKR()
        global gtau ys mkva1 mkvb1 mkva2 mkvb2 htext
        str = [];  Mi=1;
        for k=1:7, 
            set(htext(k), 'Position', [gtau(end)*1.5, ys(end,k), 0]);
            if max(ys(:,k)./ys(:,8))<1/5, continue; end;
            if k<=5, x=log10(gtau(1:2)); y=log10(ys(1:2,k)); slope=(y(2)-y(1))/(x(2)-x(1)); y1=10^(slope*(0-x(1))+y(1)); %y1 = 10^interp1(log10(gtau), log10(ys(:,k)), 0.0);
            elseif k==6, [~,idx] = max(ys(:,k));  [y1, tau] = maxmkv(mkva1, mkvb1, gtau(idx-1), gtau(idx+1));
            elseif k==7, [~,idx] = max(ys(:,k));  [y1, tau] = maxmkv(mkva2, mkvb2, gtau(idx-1), gtau(idx+1)); end
            switch k
                case 1, str = [str, sprintf('Q=%.3f (\\prime\\prime);  ', y1/sqrt(3))];
                case 2, str = [str, sprintf('N=%.6f (\\circ/h^{1/2});  ', y1/60)];
                case 3, str = [str, sprintf('B=%.4f (\\circ/h);  ', y1*3/2)];
                case 4, str = [str, sprintf('K=%.6f (\\circ/h^{3/2});  ', y1*sqrt(3)*60)];
                case 5, str = [str, sprintf('R=%.4f (\\circ/h^2);  ', y1*sqrt(2)*3600)];
                case {6,7}, str = [str, sprintf('M%d=%.4f (\\circ/h)@%.3f (s);  ', Mi, y1/0.62, tau/1.9)]; Mi=Mi+1;
            end
        end
        title(str);
        
	function s = ararmkv(a, b)
        global gtau
        s = sqrt(1./(1/a*gtau.^-1+1/b*gtau.^1));
        
	function [m, tau] = maxmkv(a, b, t1, t2)
        t = linspace(t1,t2);  
        s = sqrt(1./(1/a*t.^-1+1/b*t.^1));
        [m, idx] = max(s);
        tau = t(idx);