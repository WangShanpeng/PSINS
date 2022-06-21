function test_particle_filter_example_linear
% 建模描述参见'捷联惯导算法与组合导航原理'书练习题47.
    Ts = 0.1;    % parameter setting
    h0 = 1000; v0 = 10;  Xk = [h0; v0];
    Qt = 0.01;  Rk = 10^2;  Qk = [0,0;0,Qt*Ts];
    tpara.Ts = Ts;
    len = 500;
    X = zeros(len,2); Z = zeros(len,1);
    for k=1:len  % model simulation
        Xk = pffx(Xk, tpara) + sqrt(Qk)*randn(2,1);   X(k,:) = Xk';
        Zk = pfhx(Xk, tpara) + sqrt(Rk)*randn(1);   Z(k,:) = Zk;
    end
    %% particle filter
    pf = pfinit([h0+10;v0+1], [50;10], Qk, Rk, 1000);
    pf.fx = @pffx; pf.hx = @pfhx;  pf.tpara = tpara;
    Xpf = X; sPpf = X;
    for k=1:len 
%         pf = pfupdate(pf, Z(k));
        pf = pfupdate(pf);
        if mod(k,1)==0
            pf = pfupdate(pf, Z(k), 'M');
        end
        if k>200
            aa=1;
        end
        % myfig,subplot(211),hist(pf.particles(1,:));subplot(212),hist(pf.particles(2,:));
        Xpf(k,:) = pf.xk';  sPpf(k,:) = sqrt(diag(pf.Pxk)');
    end
    myfig,subplot(211),hist(pf.particles(1,:));subplot(212),hist(pf.particles(2,:));
    myfig;
    subplot(321), plot([X(:,1),Xpf(:,1)]); xygo('k','h / m'); legend('h real', 'h est');
    subplot(323), plot([X(:,2),Xpf(:,2)]); xygo('k','vel / m/s'); legend('vel real', 'vel est');
    subplot(325), plot(Z); xygo('k', 'dist meas / m');
    subplot(322), plot([Xpf(:,1)-X(:,1),sPpf(:,1)]); xygo('k','h err / m'); legend('est err', 'est std');
    subplot(324), plot([Xpf(:,2)-X(:,2),sPpf(:,2)]); xygo('k','vel err / m/s'); legend('est err', 'est std');

function X = pffx(X, tpara)  % state model
    Ts = tpara.Ts;
    h = X(1);  v = X(2);
    v1 = v;
    h = h + (v+v1)/2*Ts; v = v1;
    X = [h; v];

function Z = pfhx(X, tpara)   % measurement model
    h = X(1);
    Z = h;
