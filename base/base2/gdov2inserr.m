function davp = gdov2inserr(avp, dgn, isfig)
% Gravity abnomal & Deflection of vertical (DOV) to INS error evaluation.
%
% Prototype: dpos = dov2inserr(avp, dgn)
% Inputs: avp - INS AVP, always att=0
%         dgn - Deflection of vertical (DOV) & gravity abnomal array
%         isfig - figure flag
% Output: davp - INS error
%
% Example
%     avp1 = extrapos(avp,1000);
%     txtfile('posdata.txt', '%.8f %.8f %.2f', [avp1(:,8)/glv.deg,avp1(:,7)/glv.deg,avp1(:,9)]);
%     dgn = [gdovshow(),avp1(:,end)];
%     davp = gdov2inserr(avp(1:10:end,:), delbias(dgn,dgn(1,1:3),1:3), 1);
%
% See also  pos2dxyz, dovshow.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/08/2022
    len = length(avp);
    avp = [zeros(len,3),avp(:,end-6:end)];
    ts = diff(avp(1:2,end));
    fn = diff(avp(:,4:6))/ts; fn = [fn(1,:); fn];
    kk = 1;  gnlen = length(dgn);
    X = zeros(15,1);
    davp = zeros(len,15+1);
    timebar(1, len, 'DOV to INS error processing.');
    for k=1:len
        t = avp(k,end);
        ins = insinit(avp(k,:)', 1);
        ins.fn = fn(k,:)';
        Fk = expm(etm(ins)*ts);
        while t>dgn(kk,end) && kk<gnlen, kk=kk+1; end
        X(13:15) = dgn(kk,1:3)';
        X = Fk*X;
        davp(k,:) = [X; t]';
        timebar;
    end
    if nargin<3, isfig=1; end
    if isfig==1, inserrplot(davp); end
