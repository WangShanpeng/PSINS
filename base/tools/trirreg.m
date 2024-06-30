function ir = trirreg(xyz, L10, isfig)
% Calculation of track irregularity.
%
% Prototype: ir = trirreg(xyz, L10, isfig)
% Inputs: xyz - xyz postion or pos=[lat,lon,hgt]
%         L10 - span, default 10m
%         isfig - figure flag
% Output: ir - irregularity of [leveling,height,distance]
%
% See also  distance.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/03/2024
    if nargin<2, L10=10; end
    L5 = L10/2;
    xyz = norep(xyz,1:3);
    if max(abs(xyz(:,1)))<pi/2, xyz = pos2dxyz(xyz); end  % xyz=[lat,lon,hgt]
    dis = cumsum(normv(diffs(xyz(:,1:3))));
    len = length(dis); iL10=1; iL5=1; ir=zeros(len,2);
    for k=1:len
        while dis(iL10)-dis(k)<L10, iL10=iL10+1; if iL10>len,break; end, end
        if iL10>len,break; end
        while dis(iL5)-dis(k)<L5, iL5=iL5+1; end
        xyz1 = xyz(k,:); xyz2 = xyz(iL5,:); xyz3 = xyz(iL10,:);
        dis1 = dis(k);   dis2 = dis(iL5);   dis3 = dis(iL10); 
        ir(iL5,:) = [(xyz1(1)-xyz2(1))*(xyz3(2)-xyz2(2))-(xyz3(1)-xyz2(1))*(xyz1(2)-xyz2(2)), ...
                     (xyz1(3)-xyz2(3))*(dis3-dis2)-(xyz3(3)-xyz2(3))*(dis1-dis2) ];
    end
%     ir = no0([abs(ir)/L10, dis],1:2);
    ir = no0([(ir)/L10, dis],1:2);
    if nargin<3, isfig=1; end
    if isfig==1
        myfig, plot(ir(:,3), ir(:,1:2));
        xygo('distance / m', sprintf('irregularity (L=%.0fm)',L10));  legend('level', 'height');
    end
    
