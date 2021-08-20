function [att, p] = attfusion(att1, p1, att2, p2)
% Attitude fusion.
%
% Prototype: [att, p] = attfusion(att1, p1, att2, p2)
% Inputs: att1, p1 - attitude estimation and covariance by methods 1
%         att2, p2 - attitude estimation and covariance by methods 2
% Outputs: att, p - estimation and variance fusion
%
% See also  fusion, kfupdate, POSFusion.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/12/2013
    att = att1;
	for k=1:size(att1, 1)
    	pf = p1(k,:)./(p1(k,:)+p2(k,:));
    	q1k = a2qua(att1(k,:)');
    	phi = qq2phi(a2qua(att2(k,:)'), q1k);
        att(k,:) = q2att( qaddphi(q1k,phi.*pf') );
	end
    p = p1.*p2./(p1+p2);

