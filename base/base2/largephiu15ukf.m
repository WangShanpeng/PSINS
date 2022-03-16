function Xk = largephiu15ukf(Xk_1, ins)
% Large heading misalignment angle error model(Ref. my postdoctoral report P66).
% The 15-state 'Xk' includes [phi; dvn; dpos; eb; db].
%
% Prototype: Xk = largephiu15ukf(Xk_1, ins)
% Inputs: Xk_1 - state at previous time
%         ins - ins struct
% Output: Xk - current state
%
% See also  afamodel, ukf, largephiu15ekf.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/02/2022
global glv
    phi0 = Xk_1(1:3); dvn0 = Xk_1(4:6); dpos0 = Xk_1(7:9); eb0=Xk_1(10:12); db0 = Xk_1(13:15);
    dwnie = [0; -glv.wie*ins.eth.sl*dpos0(1); glv.wie*ins.eth.cl*dpos0(1)];
    dwnen = [ -dvn0(2)/ins.eth.RMh+ins.vn(2)*dpos0(3)/ins.eth.RMh^2;
        dvn0(1)/ins.eth.RNh-ins.vn(1)*dpos0(3)/ins.eth.RNh^2;
        ins.eth.tl*dvn0(1)/ins.eth.RNh+ins.vn(1)/ins.eth.cl^2/ins.eth.RNh-ins.vn(1)*ins.eth.tl*dpos0(3)/ins.eth.RNh^2 ];
    dwnin = dwnie + dwnen;
    sz = sin(phi0(3)); cz = cos(phi0(3));
    CA = [ 1-cz, -sz, phi0(2);  sz, 1-cz, -phi0(1);  -phi0(1)*sz-phi0(2), phi0(1)*cz,0 ];
    CB = [ 1-cz, sz, -phi0(2)*cz-phi0(1)*sz;  -sz,1-cz,-phi0(2)*sz+phi0(1)*cz; phi0(2),-phi0(1),0 ];
    Cz = [cz,sz,0; -sz,cz,0; 0,0,1];
    dphi = CA*ins.eth.wnin + Cz*dwnin - ins.Cnb*eb0;
    ddvn = CB*ins.fn + Cz'*ins.Cnb*db0 - cross((2*dwnie+dwnen),ins.vn)-cross((2*ins.eth.wnie+ins.eth.wnen),dvn0);
    ddpos = [ dvn0(2)/ins.eth.RMh-ins.vn(2)*dpos0(3)/ins.eth.RMh^2;
        dvn0(1)/ins.eth.cl/ins.eth.RNh+ins.vn(1)*dpos0(1)*ins.eth.tl/ins.eth.cl/ins.eth.RNh-ins.vn(1)*dpos0(3)/ins.eth.cl/ins.eth.RNh^2;
        dvn0(3) ];
    Xk = Xk_1 + [dphi; ddvn; ddpos; zeros(6,1)]*ins.nts; 
