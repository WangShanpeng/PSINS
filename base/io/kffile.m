function [xk, pk, zk, rk, sk] = kffile(kfres, q, r, t0)
% The disassemble of KF result array (from C++\CFileRdWt& operator<<(CKalman &kf)).
%
% Prototype: [xk, pk, zk, rk, sk] = kffile(kfres, q, r, t0)
% Inputs: kfres - KF result
%         q,r - state/measure dimension
%         t0 - shown as initial 0-time
% Outputs: xk,pk,zk,rk,sk - disassemble outputs
% 
% Example:
%     psinstypedef(196);
%     [xk,pk,zk,rk,sk] = kffile('kf.bin', 28,6,0);
%     kfplot(xk,pk,1:19);
%     xpplot(xk,pk,[],1,'dKG');
%     xpplot(xk,pk,[],1,'dKA');
%     rvpplot(zk,rk);
%     stateplot(sk);
% 
% See also  psinstypedef, kfplot, rvpplot, stateplot, igkfplot, igplot, binfile.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/12/2020
    if ischar(kfres)
        kfres = binfile(kfres,(q+r)*2+2);
    end
    if nargin<4; t0=kfres(1,end); end
    kfres(:,end) = kfres(:,end) - t0;
    switch nargout
        case 1,  % xkpk = kffile(kfres, q, r)
        xk = kfres(:,[1:2*q,end]);
        case 2,  % [xkpk, zrk] = kffile(kfres, q, r)
        xk = kfres(:,[1:2*q,end]);
        pk = kfres(:,[2*q+1:2*q+2*r,end]);
        case 3,  % [xkpk, zrk, sk] = kffile(kfres, q, r)
        xk = kfres(:,[1:2*q,end]);
        pk = kfres(:,[2*q+1:2*q+2*r,end]);
        zk = kfres(:,end-1:end);
        case 5,  % [xk, pk, zk, rk, sk] = kffile(kfres, q, r)
        xk = kfres(:,[1:q,end]);
        pk = kfres(:,[q+1:2*q,end]);
        zk = kfres(:,[2*q+1:2*q+r,end]);
        rk = kfres(:,[2*q+r+1:2*q+2*r,end]);
        sk = kfres(:,end-1:end);
    end
    