function [idx, v4, cbsV] = maxtetra(vertexes)
% For some vertexes (no less than 4), calculate the max volume of tetrahedron.
%
% Prototype:  [idx, v4] = maxtetra(vertexes)
% Input: vertexes - = [ x1, y1, z1;
%                       x2, y2, z2;
%                       ...        ]
% Outputs: idx - vertex index for the max volume
%          v4 - the max volume 4-vertexes
%
% Example1:
%    vs = [0 0 0; 1 0 0; 0 1 0; 0 0 1];  [idx, v4, cbsV] = maxtetra(vs)
%
% Example2:
%    vs = randn(8, 3);  [idx, v4] = maxtetra(vs);  idx=idx([1,2,3,4,1, 2,4, 3,1]);
%    figure, plot3(vs(:,1),vs(:,2),vs(:,3),'o', vs(idx,1),vs(idx,2),vs(idx,3),'-*r');
%
% See also  DOP, satPosVel, rimucfg.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/05/2022
    cbs = combntns(1:length(vertexes),4);
    a = vertexes(cbs(:,2),:) - vertexes(cbs(:,1),:);  % side a,b,c with common vertex
    b = vertexes(cbs(:,3),:) - vertexes(cbs(:,1),:);
    c = vertexes(cbs(:,4),:) - vertexes(cbs(:,1),:);
    ab = [a(:,2).*b(:,3)-a(:,3).*b(:,2), -a(:,1).*b(:,3)+a(:,3).*b(:,1), a(:,1).*b(:,2)-a(:,2).*b(:,1)];
    V = abs(ab(:,1).*c(:,1)+ab(:,2).*c(:,2)+ab(:,3).*c(:,3))/6;  % volume Eq. V = |(aXb).c|/6
    [~, idx] = max(V);
    idx = cbs(idx,:)';
    v4 = vertexes(idx,:);
    if nargout>2
        [~, idx] = sort(V,'descend');
        cbsV = [cbs(idx,:), V(idx)];
    end
    