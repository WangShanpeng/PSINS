function ir = trirreg1(xyz, dist, isfig)
% Calculation of track irregularity by specific distance array.
%
% Prototype: ir = trirreg1(xyz, dist, isfig)
% Inputs: xyz - xyz postion or pos=[lat,lon,hgt]
%         dist - specific diatance array
%         isfig - figure flag
% Output: ir - irregularity of [leveling,height,distance]
%
% See also  trirreg.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/06/2024
    xyz = norep(xyz,1:3);
    if max(abs(xyz(:,1)))<pi/2, xyz = pos2dxyz(xyz); end  % xyz=[lat,lon,hgt]
    dis = norep(cumsum(normv(diffs(xyz(:,1:3)))),1);
    idx1 = interp1(dis, 1:length(dis), dist(:,1), 'nearest');
    idx2 = interp1(dis, 1:length(dis), dist(:,2), 'nearest');
    idx3 = interp1(dis, 1:length(dis), dist(:,3), 'nearest');
    inan = isnan(idx1) & isnan(idx2) & isnan(idx3);
    idx1(inan) = []; idx2(inan) = []; idx3(inan) = [];
    len = length(idx1); ir=zeros(len,3);
    for k=1:len
        xyz1 = xyz(idx1(k),:); xyz2 = xyz(idx2(k),:); xyz3 = xyz(idx3(k),:);
        dis1 = dis(idx1(k));   dis2 = dis(idx2(k));   dis3 = dis(idx3(k)); 
        ir(k,:) = [(xyz1(1)-xyz2(1))*(xyz3(2)-xyz2(2))-(xyz3(1)-xyz2(1))*(xyz1(2)-xyz2(2)), ...
                   (xyz1(3)-xyz2(3))*(dis3-dis2)-(xyz3(3)-xyz2(3))*(dis1-dis2), ...
                   dis2  ];
        ir(k,1:2) = ir(k,1:2)/(dis3-dis1);
    end
    if nargin<3, isfig=1; end
    if isfig==1
        myfig, plot(ir(:,3), ir(:,1:2));
        xygo('distance / m', sprintf('irregularity / m (L=%.1fm)',mean(dis(idx3)-dis(idx1))));  legend('level', 'height');
    end
    
