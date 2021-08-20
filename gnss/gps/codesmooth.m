function prsm = codesmooth(tpPRN, pr, carrier, N, isfigure)
% Smoothing of pseudorange with carrier phase (code smoothing), but cycle
% slip is not considered.
%
% Prototype: prsm = codesmooth(tpPRN, pr, carrier, N, isfigure)
% Inputs: tpPRN - tpPRN = [tp, PRN]
%         pr - pseudorange
%         carrier - carrier phase meas
%         N - average epoch num
%         isfigure - flag for figure or not
% Outputs: prsm - pseudorange smoothing
%
% See also  lsPos.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/08/2015
    if nargin<5, isfigure=0; end
    clr = 'bgrcmyk';
    tp = tpPRN(:,1); PRNs = unique(tpPRN(:,2));
    prsm = zeros(size(tp));
    for n=PRNs(1):PRNs(end)
        idx=(tpPRN(:,2)==n&pr~=0);
        tpi=tp(idx); pri=pr(idx); cfi=carrier(idx);
        if length(tpi)<N, continue; end
        prsmi = cfi + filter(ones(N,1)/N, 1, pri-cfi);  % FIR average smoothing
        for k=1:N-1
            prsmi(k) = cfi(k) + mean(pri(1:k)-cfi(1:k));
        end
        prsm(idx) = prsmi;
        if isfigure
            nn = mod(n-1,36)+1;
            if nn==1, myfigure; end
            k = fix((nn-0.1)/6)+1;
            subplot(3,2,k); plot(tpi,[pri-cfi,prsmi-cfi],['-x',clr(mod(n-1,5)+1)]); xygo('Code Smooth');
%             subplot(2,2,k); plot(tpi,pri-prsmi,['-x',clr(mod(n-1,5)+1)]); xygo('Code Smooth');
            text(tpi(end), pri(end)-cfi(end), num2str(n));
        end
    end

