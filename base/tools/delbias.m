function [x, b1] = delbias(x, b, clm)
% Delete bias.
%
% Prototype:  [x, b1] = delbias(x, b, clm)
% Inputs: x_in - input data with bias
%         b - bias
%         clm - data column to delete bias
% Outputs: x_out - output data with no bias
%          b1 - bias output
%
% See also  deltrend, imudeldrift, mulsf, adddt, sumn, meann.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/07/2016, 03/07/2018, 24/03/2022 
    if nargin==1
        clm=1:size(x,2); b = mean(x(:,clm));   %  delbias(x)  % clm=1:size(x,2)
    end
    if nargin==3
        if isempty(b); b = mean(x(:,clm)); end     %  delbias(x, [], clm)
        if length(b)==1; b = repmat(b,clm,1); end    %  delbias(x, [bbb...], clm)
        for k=1:length(clm)
            x(:,clm(k)) = x(:,clm(k)) - b(k);
        end
        return;
    end
    for k=1:size(x,2)
        if exist('b','var')
            if size(x,1)==size(b,1), x(:,k) = x(:,k)-b;         % x = [x(:,1)-b, x(:,2)-b, x(:,3)-b, ...]
            elseif k<=length(b), x(:,k) = x(:,k)-b(k); end      % x = [x(:,1)-b(1), x(:,2)-b(2), x(:,3)-b(3), ...]
            b1 = b;
        else
            b1(k,:) = mean(x(:,k));
            x(:,k) = x(:,k)-b1(k,:);
        end
    end