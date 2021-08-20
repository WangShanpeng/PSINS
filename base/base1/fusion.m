function [x, p] = fusion(x1, p1, x2, p2, isatt)
% Data fusion.
%
% Prototype: [x, p] = fusion(x1, p1, x2, p2)
% Inputs: x1, p1 - state estimation and covariance by methods 1
%         x2, p2 - state estimation and covariance by methods 2
% Outputs: x, p - estimation and variance fusion
% Alogrithm notes:
% If p1 & p2 are covariance matrices, then:
%     x = ( p2*x1 + p1*x2 )/(p1+p2)
%     p = p1*p2/(p1+p2)
% If p1 & p2 are diagnal matrices of covariance, then:
%     x = ( p2.*x1 + p1.*x2 )./(p1+p2)
%     p = p1.*p2./(p1+p2)
%
% See also  attfusion, kfupdate, POSFusion.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/12/2013
    if nargin<4         % xpt = fusion(xpt1, xpt2, isatt)
        if nargin<3, isatt=0; end
        [m,n] = size(p1); n2 = (n-1)/2;
        [x, p] = fusion(x1(:,[1:n2]), x1(:,n2+1:end-1), p1(:,[1:n2]), p1(:,n2+1:end-1), isatt);
        x = [x,p,x1(:,end)];
        return;
    end
    [m,n] = size(p1);
    if m==n && size(x1,2)==1 % p1 is matrix P, but not diag(P) 
        x = ( p2*x1 + p1*x2 )/(p1+p2);  
        p = p1*p2/(p1+p2);
    else
        p = p1+p2;
        x = ( p2.*x1 + p1.*x2 )./p;
        p = p1.*p2./p;
    end
    if nargin<5, isatt=0; end
    if isatt==1
        att = attfusion(x1(:,1:3), p1(:,1:3), x2(:,1:3), p2(:,1:3));
        x(:,1:3) = att;
    end
