function lstyle(varargin)
% Set line color/linestyle/marker/linewidth.
%
% Prototype: lstyle(varargin)
% Inputs: color/linestyle/marker/linewidth strings
% Output: N/A
% 
% See also lneg, lmul, laddy, lmc.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/01/2023
    linewidth = 1;
    if nargin==1, linewidth=varargin{end}; end   % lstyle(linewidth)
    if nargin<=1
        varargin = {'1.-', '2.-', '3.-', '4.-', '5.-', '6.-', '7.-', linewidth};
    end
    if isnumeric(varargin{end}), linewidth=varargin{end}; varargin{end}=[]; end
    if length(linewidth)==1, linewidth=repmat(linewidth,7,1); end
    ob = findobj(gca);
    if isempty(ob), error('No line found.');  end
    kk = 1;
    for k=length(ob):-1:1
        lin = get(ob(k));
        if ~isfield(lin, 'XData'), continue; end
        if kk>length(varargin), break; end
        sty = varargin{kk};
        if length(lin.XData)>1
            clr = [];
            switch sty(1)
                case '1', clr = [ 0     0     1];
                case '2', clr = [ 0     0.5   0];
                case '3', clr = [ 1     0     0];
                case '4', clr = [ 0     0.75  0.75];
                case '5', clr = [ 0.75  0     0.75];
                case '6', clr = [ 0.75  0.75  0];
                case '7', clr = [ 0.25  0.25  0.25];
                case 'b', clr = [ 0     0     1];
                case 'g', clr = [ 0     1     0];
                case 'r', clr = [ 1     0     0];
                case 'c', clr = [ 0     1     1];
                case 'm', clr = [ 1     0     1];
                case 'y', clr = [ 1     1     0];
                case 'k', clr = [ 0     0     0];
                case 'w', clr = [ 1     1     1];
            end
            mk = 'none';
            switch sty(2)
                case {'.','o','x','+','*','s','d','v','^','<','>','p','h'}, mk = sty(2);
            end
            ls = '';
            switch sty(3:end)
                case {'-',':','-.','--'}, ls = sty(3:end);
            end
            if ~isempty(clr), set(ob(k),'Color',clr); end
            if ~isempty(mk), set(ob(k),'Marker', mk); end
            if ~isempty(ls), set(ob(k),'LineStyle', ls); end
            set(ob(k),'LineWidth', linewidth(kk));
            kk = kk+1;
        end
    end
    
