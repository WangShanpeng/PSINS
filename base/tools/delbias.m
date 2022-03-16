function [x, b1] = delbias(x, b, clm)
% Delete bias.
%
% Prototype:  out = delbias(in, b, clm)
% Inputs: in - input date with bias
%         b - bias
%         clm - data column to delete bias
% Outputs: out - output date with no bias
%          b1 - bias output
%
% See also  imudeldrift, adddt, sumn, meann.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/07/2016, 03/07/2018 
    if nargin==3
        if length(b)==0; b = mean(x(:,clm)); end    %  delbias(x, [], clm)
        if length(b)==1; b = repmat(b,clm,1); end    %  delbias(x, b, clm)
        for k=1:length(clm)
            x(:,clm(k)) = x(:,clm(k)) - b(k);
        end
        return;
    end
    for k=1:size(x,2)
        if exist('b','var')
            if k<=length(b), x(:,k) = x(:,k)-b(k); end
            b1 = b;
        else
            b1(k,:) = mean(x(:,k));
            x(:,k) = x(:,k)-b1(k,:);
        end
    end