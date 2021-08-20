function [sigma, tau, Err] = avars(y0, tau0)
% Calculate Allan variance for multi-column gyro data.
%
% Prototype: [sigma, tau, Err] = avars(y0, tau0, isfig)
%
% See also  avar, avar2.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/12/2019
    str = ['-+b'; '-xk'; '-*m'; '-oy'];
    myfig;
    sigma = []; tau = []; Err = [];
    sstr = [];
    for k=1:size(y0,2);
        [sigma(:,k), tau(:,k), Err(:,k)] = avar(y0(:,k), tau0, 0, 0);
        loglog(tau(:,k), sigma(:,k), str(k,:), tau(:,k), [sigma(:,k).*(1+Err(:,k)),sigma(:,k).*(1-Err(:,k))], 'r--'); hold on
        sstr = [sstr,' / ',str(k,:)];
    end
    grid on;
    xlabel('\itt \rm/ s'); ylabel('\it\sigma_A\rm( \tau ) \rm (\circ)/h');
    title(sstr);

