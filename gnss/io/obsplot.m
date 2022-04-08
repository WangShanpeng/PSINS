function obsplot(tpPRN, obs, ytext)
% Satellite observation analysis and plot.
%
% Prototype: [obsplot(tpPRN, obs)
% Inputs: tpPRN - tpPRN = [tp, PRN]
%         obs - observations
%         ytext - ylabel text
%
% See also  satplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/07/2015
    clr = 'bgrcmyk';
    t0 = tpPRN(1,1);
    tp = tpPRN(:,1)-t0; PRN = tpPRN(:,2); uniPRN = unique(PRN);
    myfigure
    len = length(uniPRN); clr = repmat(clr,1,fix(len/length(clr))+1);
    for k=1:len
        idx = (PRN==uniPRN(k));  ktp = tp(idx);  kobs = obs(idx,:);
        plot(ktp,kobs,['-x',clr(k)]);
        text(ktp(end), kobs(end,:), num2str(uniPRN(k))); hold on;
    end
    if nargin<3, ytext = 'obs'; end
    xygo(sprintf('\\itt\\rm / s (+%0.f)',t0), ytext);