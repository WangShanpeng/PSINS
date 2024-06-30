function [att, rv] = attinterp1(att, t, method)
% att linear interpolation. 
%
% Prototype: att = attinterp1(att, t, method)
% Inputs: att - input attitude
%         t - time tag
%         method - 'nearest'/'linear' etc, see interp1
% Output: att - interpolated attitude
%
% See also  avpinterp1, att2c, attinterp.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/11/2019
    if ~exist('method', 'var'), method = 'linear'; end
    i1 = find(t>att(1,end),1,'first');
    i2 = find(t<att(end,end),1,'last');
    t = t(i1:i2)';
%     att(:,1:3) = att2c(att(:,1:3));
%     att = [interp1(att(:,end), att(:,1:end-1), t, method), t];
%     att(:,1:3) = iatt2c(att(:,1:3));
    qnb = a2quaBatch(att(:,1:3));
    rv = [0,0,0; qq2rvBatch(qnb)];
    rv = interp1(att(:,end), rv, t, method);  % myfig, plot([qnb,normv(qnb)]);
    qnb = repmat(qnb(1,:),length(rv),1);
    for k=1:length(rv)-1
        qnb(k+1,:) = qupdt(qnb(k,:)',rv(k,:)')';
    end
    att = [q2attBatch(qnb), t];
    
function rv = qq2rvBatch(q1, q0)
% rv = q2rv(qmul(qconj(q0),q1));
    if nargin==2, q2=[q0(:,1),-q0(:,2:4)]; q1=q;
    else q2 = q1(2:end,:); q1 = [q1(1:end-1,1),-q1(1:end-1,2:4)]; end
    q = [ q1(:,1) .* q2(:,1) - q1(:,2) .* q2(:,2) - q1(:,3) .* q2(:,3) - q1(:,4) .* q2(:,4), ...
          q1(:,1) .* q2(:,2) + q1(:,2) .* q2(:,1) + q1(:,3) .* q2(:,4) - q1(:,4) .* q2(:,3), ...
          q1(:,1) .* q2(:,3) + q1(:,3) .* q2(:,1) + q1(:,4) .* q2(:,2) - q1(:,2) .* q2(:,4), ...
          q1(:,1) .* q2(:,4) + q1(:,4) .* q2(:,1) + q1(:,2) .* q2(:,3) - q1(:,3) .* q2(:,2) ];
	idx=q(:,1)<0; q(idx,:) = -q(idx,:);
    n2 = acos(q(:,1));
    k = repmat(2,length(n2),1);
    idx=n2>1e-40; k(idx,:) = 2*n2(idx,:)./sin(n2(idx,:));
    rv = [k.*q(:,2),k.*q(:,3),k.*q(:,4)];
    