function [lnk,idx] = obsEphLink(obs, eph)
% Create link between obs records and eph records.
% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/08/2015
    tp = obs(:,3)*1e7+obs(:,2); idxeph = zeros(length(tp),2); toe = eph(:,1)*1e7+eph(:,2);  %2->14
    for k = 1:length(tp)
        [mm,idx] = min(abs(tp(k)-toe)); idxeph(k,:) = [idx, obs(k,2)-eph(idx,2)];
        if eph(idx,27)==1, idxeph(k,1) = -1; continue; end % not SatHealth
        if mm>7200, idxeph(k,1) = -2; continue; end
    end
    % delete error data
    idx = idxeph(:,1)>0;
    lnk = idxeph(idx,:);
    