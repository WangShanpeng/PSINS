function varargout = datacuts(data0, varargin)
% Extract data between time tags varargin{k} & varargin{k+1}.
%
% Prototype: varargout = datacuts(data0, varargin)
% Inputs: data0 - input data, whose last column should be time index
%         varargin - start & end time tags
% Outputs: varargout - output data
%
% See also  datacut, datahalf, setvals.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/12/2023
    if nargin==2   % varargout = datacuts(data0, [t0,t1,t2,...])
        t = varargin;  varargin = [];
        for k=1:length(t{1}), varargin{k} = t{1}(k);  end
    end
    if nargout==length(varargin)
        varargin = [{-inf}, varargin];
    elseif nargout==length(varargin)+1
        varargin = [{-inf}, varargin, {inf}];
    end
    nout = nargout;
    if nargout==1, v=[]; nout=length(varargin)-1; end
    for k=1:nout
        varargout{k} = datacut(data0, varargin{k}, varargin{k+1});
        if mod(k-1,2)==0 && exist('v','var'); v=[v; varargout{k}]; end
    end
    if exist('v','var')
        varargout{1} = v; 
    end

