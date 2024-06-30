function [sigma, tau, Err] = avar(y0, tau0, str, isfig)
% Calculate Allan variance.
%
% Prototype: [sigma, tau, Err] = avar(y0, tau0, str, isfig)
% Inputs: y0 - data (gyro in deg/hur; acc in mg)
%         tau0 - sampling interval
%         str - string for ylabel
%         isfig - figure flag
% Outputs: sigma - Allan variance
%          tau - Allan variance correlated time
%          Err - Allan variance error boundary
%
% Examples: 
%     x = randn(100000,1) + 0.00001*[1:100000]'; ts = 0.1;
%     [sigma, tau, Err] = avar(x, ts);       % gyro in deg/h
%     [sigma, tau, Err] = avar(x, ts, 2);    % gyro in deg/h, plus cumsum angle plot
%     [sigma, tau, Err] = avar(x, ts, 'mg'); % acc in mg
%
% See also  avarsimu, avarfit, avar2, avars, avarimu, oavar, tavar, meann, hrgbi.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/08/2012, 16/12/2019, 30/5/2021, 5/7/2023
global glv
    N = length(y0);
    y = y0; NL = N;
    for k = 1:log2(N)
        sigma(k,1) = sqrt(1/(2*(NL-1))*sum((y(2:NL)-y(1:NL-1)).^2)); % diff&std
        tau(k,1) = 2^(k-1)*tau0;      % correlated time
        Err(k,1) = 1/sqrt(2*(NL-1));  % error boundary
        NL = floor(NL/2);
        if NL<3
            break;
        end
        y = 1/2*(y(1:2:2*NL) + y(2:2:2*NL));  % mean & half data length
    end
    if nargin<4, isfig=1; end
    if nargin<3, str=[]; end
    if (~isempty(str)) && isnumeric(str), isfig=str; str=[]; end  % avar(y0, tau0, 1)
    if isfig && glv.isfig==1
        figure('Color','White');
        if isempty(str), str = '/ ( (\circ) / h )'; elseif strcmp(str,'mg')==1, str='/ mg'; end
        subplot(211), plot(tau0*(1:N)', y0); grid
        xlabel('\itt \rm/ s'); ylabel(['\itY \rm',str]); title(sprintf('Mean: %.6f', mean(y0)));
        subplot(212), 
        loglog(tau, sigma, '-+', tau, [sigma.*(1+Err),sigma.*(1-Err)], 'r--'); grid
        idx = strfind(str,'/');
        if ~isempty(idx),   str = str(idx(1):end);
        else                str = [];
        end
        str = strcat('\it\sigma_A\rm( \tau ) \rm',str);
        xlabel('\it\tau \rm/ s'); ylabel(str);
    end
	if isfig==2  % re-plot subplot(211), the cumsum angle
        my0 = mean(y0(1:fix(length(y0)/100)));
%         ang = cumsum(y0-my0)*tau0/3600;
        ang = cumsum(y0-mean(y0))*tau0/3600;
        subplot(211); [ax,h1,h2] = plotyy(tau0*(1:N)', y0, tau0*(1:N)', ang*60); grid on;
        xlabel('\itt \rm/ s'); 
        ylabel(ax(1), '\it\omega \rm/ ( (\circ) / h )');  ylabel(ax(2), '\Delta\it\theta \rm/ (\prime)');
        title(sprintf('Mean: %.6f ( (\\circ) / h )', my0));
    end

