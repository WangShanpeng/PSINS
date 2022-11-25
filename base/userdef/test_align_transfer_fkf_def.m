function out = test_align_transfer_fkf_def(tag, varargin)
% See also  test_align_transfer_fkf, kfinit, kffk, kfhk, kfplot.
global glv psinsdef
switch tag
	case psinsdef.kfinittag,
        [nts, imuerr] = setvals(varargin);
        kf.Phikk_1 = eye(15); 
        kf.Hk = zeros(6,15); kf.Hk(1:6,[1:3,7:9]) = eye(6);
        kf.Qt = diag([imuerr.web; zeros(3,1); imuerr.wdb; zeros(6,1)])^2; kf.Qk=kf.Qt*nts;
        kf.Rk = diag([[1;1;1]*glv.min; [.1;.1;.1]])^2;
        kf.Pxk = diag([[10;10;10]*glv.deg; imuerr.eb; [10;10;10]; imuerr.db; [10;10;10]*glv.deg])^2;
        out = kf;
    case psinsdef.kffktag,
        % 15-states: phi(3), eb(3), dV(3), db(3), mu(3)
    case psinsdef.kfhktag,
        % Hk = zeros(6,15); kf.Hk(1:6,[1:3,7:9]) = eye(6);
    case psinsdef.kfplottag,
        [xkpk, res, imuerr, mub, xkpk1, xkpk2, xkpkf] = setvals(varargin);
        t = xkpk(:,end); len = length(t);
	if size(xkpkf,2)==12
        myfig,
        subplot(321),plot(t, xkpk(:,1:3)/glv.min), xygo('phi'), plot(t,res(:,1:3)/glv.min,'-.'), plot(t, xkpkf(:,1:3)/glv.min, 'linewidth',2),
        subplot(322),plot(t, xkpk(:,4:6)/glv.dph), xygo('eb'),  plot(t,repmat(imuerr.eb,1,len)/glv.dph,'-.'), plot(t, xkpkf(:,4:6)/glv.dph, 'linewidth',2),
        subplot(323),plot(t, xkpk(:,7:9)), xygo('dV');          plot(t,res(:,4:6),'-.'),  plot(t, xkpk2(:,[7:9]), '-.', 'linewidth',2)
        subplot(324),plot(t, xkpk(:,10:12)/glv.ug), xygo('db'), plot(t,repmat(imuerr.db,1,len)/glv.ug,'-.'),  plot(t, xkpk2(:,[10:12])/glv.ug, '-.', 'linewidth',2)
        subplot(325),plot(t, xkpk(:,13:15)/glv.min), xygo('mu'), plot(t,repmat(mub,1,len)/glv.min,'-.'),  plot(t, xkpk1(:,[7:9])/glv.min, '-.', 'linewidth',2)
        myfig,
        xkpk(:,15+[1:15]) = sqrt(xkpk(:,15+[1:15]));  xkpkf(:,6+[1:6]) = sqrt(xkpkf(:,6+[1:6]));
        xkpk1(:,9+[1:9]) = sqrt(xkpk1(:,9+[1:9]));  xkpk2(:,12+[1:12]) = sqrt(xkpk2(:,12+[1:12]));
        subplot(321),plot(t, xkpk(:,15+[1:3])/glv.min), xygo('phi'); plot(t, xkpkf(:,6+[1:3])/glv.min, '-.', 'linewidth',2),
        subplot(322),plot(t, xkpk(:,15+[4:6])/glv.dph), xygo('eb'); plot(t, xkpkf(:,6+[4:6])/glv.dph, '-.', 'linewidth',2)
        subplot(323),plot(t, xkpk(:,15+[7:9])), xygo('dV');  plot(t, xkpk2(:,12+[7:9]), '-.', 'linewidth',2)
        subplot(324),plot(t, xkpk(:,15+[10:12])/glv.ug), xygo('db');  plot(t, xkpk2(:,12+[10:12])/glv.ug, '-.', 'linewidth',2)
        subplot(325),plot(t, xkpk(:,15+[13:15])/glv.min), xygo('mu');  plot(t, xkpk1(:,9+[7:9])/glv.min, '-.', 'linewidth',2)
    else
        myfig,
        subplot(321),plot(t, xkpk(:,1:3)/glv.min), xygo('phi'), plot(t,res(:,1:3)/glv.min,'-.'), plot(t, xkpkf(:,1:3)/glv.min, '-.', 'linewidth',2),
        subplot(322),plot(t, xkpk(:,4:6)/glv.dph), xygo('eb'),  plot(t,repmat(imuerr.eb,1,len)/glv.dph,'-.'), plot(t, xkpkf(:,4:6)/glv.dph, '-.', 'linewidth',2),
        subplot(323),plot(t, xkpk(:,7:9)), xygo('dV');          plot(t,res(:,4:6),'-.'),  plot(t, xkpkf(:,[7:9]), '-.', 'linewidth',2)
        subplot(324),plot(t, xkpk(:,10:12)/glv.ug), xygo('db'), plot(t,repmat(imuerr.db,1,len)/glv.ug,'-.'),  plot(t, xkpkf(:,[10:12])/glv.ug, '-.', 'linewidth',2)
        subplot(325),plot(t, xkpk(:,13:15)/glv.min), xygo('mu'), plot(t,repmat(mub,1,len)/glv.min,'-.'),  plot(t, xkpkf(:,[13:15])/glv.min, '-.', 'linewidth',2)
        myfig,
        xkpk(:,15+[1:15]) = sqrt(xkpk(:,15+[1:15]));  xkpkf(:,15+[1:15]) = sqrt(xkpkf(:,15+[1:15]));
        subplot(321),plot(t, xkpk(:,15+[1:3])/glv.min), xygo('phi'); plot(t, xkpkf(:,15+[1:3])/glv.min, '-.', 'linewidth',2),
        subplot(322),plot(t, xkpk(:,15+[4:6])/glv.dph), xygo('eb'); plot(t, xkpkf(:,15+[4:6])/glv.dph, '-.', 'linewidth',2)
        subplot(323),plot(t, xkpk(:,15+[7:9])), xygo('dV');  plot(t, xkpkf(:,15+[7:9]), '-.', 'linewidth',2)
        subplot(324),plot(t, xkpk(:,15+[10:12])/glv.ug), xygo('db');  plot(t, xkpkf(:,15+[10:12])/glv.ug, '-.', 'linewidth',2)
        subplot(325),plot(t, xkpk(:,15+[13:15])/glv.min), xygo('mu');  plot(t, xkpkf(:,15+[13:15])/glv.min, '-.', 'linewidth',2)
    end
end
