function [phiu0, vnfit] = av2phiu(av, lti)
% Calculating&Plot yaw misalign angles from pure SINS velocity error.
%
% Prototype: [phiu0, vnfit] = av2phiu(vn, lti)
% Inputs: vn - pure SINS velocity error, in most case for static base
%         lti - latitude
% Output: phiu0 - misalignment between calculating navigation frame and real
%               navigation frame
%         vnfit - polyfit for velocity
%
% See also  vn2phiu, vn2phi.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/04/2021
global glv
    phie = []; phiu0 = []; vnfit = [];
    myfig;
    for k=1:length(av)
        att0 = [repmat(av{k}(1,1:3),length(av{k}),1),av{k}(:,end)];
        datt = avpcmp(av{k}(:,[1:3,end]), att0, 'phi');
        subplot(211), plot(datt(:,end), datt(:,1:3)/glv.sec); xygo('\phi / \prime\prime'); 
        text(datt(end,end)+10,datt(end,1)/glv.sec,sprintf('%.2f\\prime\\prime', datt(end,1)/glv.sec));
        phie(k,1) = datt(end,1);
        [phi, vnf] = vn2phiu(av{k}(:,[4:6,end]), lti);
        subplot(212), plot(av{k}(:,end), av{k}(:,5), vnf(:,end), vnf(:,2)); xygo('VN');
        text(vnf(end,end)+10,vnf(end,2),sprintf('%.2f\\prime', phi/glv.min));
        phiu0(k,1) = phi; vnfit{k} = vnf;
    end
    subplot(211), title(sprintf('std(phie) = %.2f(\\prime\\prime)', std(phie)/glv.sec));
    subplot(212), title(sprintf('std(phiu0) = %.2f(\\prime)', std(phiu0)/glv.min));
    return;

