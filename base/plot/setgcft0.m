function t0 = scft0(t0, minTh)
% Set the minimum xdata in gcf to 't0';
%
% Prototype: t0 = scft0(t0, minTh)
% Inputs: t0 - t0.
%         minTh - minimum threshold for not modify the xdata
% Output: NA
%
% See also  gcft0, xlimall, ladd.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/11/2022
global gcf_t00
    if isempty(gcf_t00), gcft0(); end
    if nargin<2, minTh=1000; end
    if nargin<1, t0=gcf_t00; end;
    if t0==0, t0=getgcft0(); end
    ax = findall(gcf, 'type', 'axes');
    Th = inf;
    for k=1:length(ax)  % find the minimum xdata
        if isempty(get(ax(k),'Tag'))
            lin = findall(ax(k), 'type', 'line');
            for kk=1:length(lin)
                t = get(lin(kk), 'xdata');
                Th = min(Th, t(1));
            end
        end
    end
    if Th<minTh, t0=0; return; end
    %%
    ax = findall(gcf, 'type', 'axes');
    for k=1:length(ax)
        if isempty(get(ax(k),'Tag'))
            lin = findall(ax(k), 'type', 'line');
            for kk=1:length(lin)
                t = get(lin(kk), 'xdata');
                set(lin(kk), 'xdata', t-t0);
            end
        end
    end

