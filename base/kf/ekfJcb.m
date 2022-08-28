function [Jcb, y] = ekfJcb(hfx, x, tpara)
% Spherical-Radial Cubature transformation.
%
% Prototype: [y, Pyy, Pxy, X, Y] = ckfCT(x, Pxx, hfx, tpara)
% Inputs: hfx - a handle for nonlinear state equation
%         x - state vector
%         tpara - some time-variant parameter pass to hfx
% Outputs: Jcb - Jacobian matrix of hfx
%          y - y = fhx(x)
%
% See also  ekf, ckf, ukf, Jacob5, alignvn_ekf.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/03/2022
    [Jcb, y] = feval(hfx, x, tpara);
