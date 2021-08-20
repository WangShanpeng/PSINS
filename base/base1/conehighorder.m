function dphim = conehighorder(wm)
% Calculation of noncommutativity error using uncompressed coning algorithm.
% Ref. YanGM. 'A new method to obtain high-order error compensation coefficients for equivalent rotation vector', 2020
%
% Prototype: dphim = conehighorder(wm)
% Input: wm - gyro angular increments
% Output: dphim - noncommutativity error compensation vector
%
% See also  conepolyn, conetwospeed, conedrift, scullpolyn, cnscl.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2020, 20/7/2020
global glv
    n = size(wm,1);
    if n<2 || n>5, error('Sub-sample number error.'); return; end
    if ~isfield('glv', 'hocoef')
        load highordercoef.mat;
        glv.hocoef = hocoef;
    end
    hoc = glv.hocoef{n};
    dphim = zeros(1,3);
    for k=1:length(hoc.K2)
        dphim = dphim + hoc.K2(k)*cross(wm(hoc.ij(k,1),:), wm(hoc.ij(k,2),:));
    end
%         dphim = ... % Uncompressed
%             232/315*cros(wm(3,:),wm(4,:)) +  46/105*cros(wm(2,:),wm(4,:)) + ...
%               18/35*cros(wm(1,:),wm(4,:)) + 178/315*cros(wm(2,:),wm(3,:)) + ...
%              46/105*cros(wm(1,:),wm(3,:)) + 232/315*cros(wm(1,:),wm(2,:));
    for k=1:length(hoc.K3)
        dphim = dphim + hoc.K3(k)*cross(wm(hoc.ijk(k,1),:), cross(wm(hoc.ijk(k,2),:), wm(hoc.ijk(k,3),:)));
    end
	for k=1:length(hoc.K4)
        dphim = dphim + hoc.K4(k)*cross(wm(hoc.ijkl(k,1),:), cross(wm(hoc.ijkl(k,2),:), cross(wm(hoc.ijkl(k,3),:), wm(hoc.ijkl(k,4),:))));
    end
 