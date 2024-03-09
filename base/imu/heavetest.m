function heavetest(az, ts, T, N)
% Heave test for acc.
%
% Prototype: heavetest(az, ts, T, N)
% Inputs: az - vetical acc.
%         ts - sampling interval
%         T - simulating time length
%         N - simulating samples
% Output: N/A
% 
% See also  avpset, poserrset.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2023
    if nargin<4, N=10; end
    if nargin<3, T=30; end
    if size(az,2)>2, % heavetest(imu, 0, T, N);
        ts = diff(az(1:2,end));
        heavetest(az(:,6)/ts, ts, T, N);
        return;
    end
    M = fix(T/ts);
    N = min(N, floor(length(az)/M));
    az = reshape(az(1:M*N), M, N);
    for k=size(az,2):-1:1, az(:,k) = az(:,k)-mean(az(:,1)); end  % 1 or k
    h = cumsum(cumsum(az))*ts^2;
    myfig, plot((1:M)*ts, h);  xygo('heave / m');
    