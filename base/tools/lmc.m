function lmc(line1, line2, unit, iscut, idx)
% Line Move & Compare:
%   <-, -> :  for x-axis move
%   ^, v   :  for y-axis move
%   -, =   :  for change parameter 'dxy' by /*10
%
% Prototype: lmc(line1, line2, unit, iscut, idx)
% Inputs: line1 - data1 = [column_data, time_tag]
%         line2 - data2 = [column_data, time_tag]
%         unit - unit for column_data
%         iscut - cut data 
%         idx - data index 
% Output: N/A
%
% Example: lmc(randn(10,1), randn(11,1));
% 
% See also ladd, avplmc, lmrs, lstyle, avarfit.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/05/2021
global kpress_para
    if nargin<4, iscut=1; end
    if nargin==1, line2=line1(:,[2,end]); unit=1; end
    if length(line2)<3, unit=line2; line2=line1(:,[2,end]); end
    if ~exist('unit', 'var'), unit=[1;1]; end
    if length(unit)==1, unit=[unit;unit]; end
    if size(line1,2)<2, line1=[line1, (1:length(line1))']; end
    if size(line2,2)<2, line2=[line2, (1:length(line2))']; end
    if iscut
        t1 = max(line1(1,end),line2(1,end));  t2 = min(line1(end,end),line2(end,end));
        line1 = datacut(line1, t1, t2);  line2 = datacut(line2, t1, t2);
    end
    if nargin==5
        lmc(line1(:,[idx,end]), line2(:,[idx,end]), unit, iscut);
        return;
    end
    kpress_para = [];
    kpress_para.dxy=1; 
    kpress_para.dx=0.0; kpress_para.dy=0.0;
    kpress_para.xy1=[line1(:,end), line1(:,1)/unit(1)]; 
    kpress_para.xy2=[line2(:,end), line2(:,1)/unit(2)]; 
    myfig;
    subplot(211), plot(kpress_para.xy1(:,1), kpress_para.xy1(:,2), 'b', kpress_para.xy2(:,1), kpress_para.xy2(:,2), 'r'); grid on;
    err = lineerr(kpress_para.xy1, kpress_para.xy2);
    subplot(212), plot(err(:,1), err(:,2), 'm'); grid on;
    hf = gcf;
    set(hf,'WindowKeyPressFcn',@keypressfcn)
    subplot(211);
    title(sprintf('dxy=%f; dx=%f, dy=%f', kpress_para.dxy, kpress_para.dx, kpress_para.dy));
    
function  keypressfcn(h, evt)
global kpress_para
    switch evt.Key
        case 'leftarrow'
            kpress_para.dx = kpress_para.dx - kpress_para.dxy;
        case 'rightarrow'
            kpress_para.dx = kpress_para.dx + kpress_para.dxy;
        case 'uparrow'
            kpress_para.dy = kpress_para.dy + kpress_para.dxy;
        case 'downarrow'
            kpress_para.dy = kpress_para.dy - kpress_para.dxy;
        case 'equal'
            if kpress_para.dxy<1e5, kpress_para.dxy = kpress_para.dxy*10; end
        case 'hyphen'
            if kpress_para.dxy>1e-5, kpress_para.dxy = kpress_para.dxy/10; end
    end
    subplot(211);
    title(sprintf('dxy=%f; dx=%f, dy=%f', kpress_para.dxy, kpress_para.dx, kpress_para.dy));
    lines = findall(gca, 'type', 'line');
    x2 = kpress_para.xy2(:,1)+kpress_para.dx;
    y2 = kpress_para.xy2(:,2)+kpress_para.dy;
    set(lines(1), 'xdata', x2);
    set(lines(1), 'ydata', y2);
    xx = get(gca, 'XLim');
    subplot(212);
    title('difference');
    err = lineerr(kpress_para.xy1, [x2,y2]);
    plot(err(:,1), err(:,2), 'r'); grid on; 
    xlim(xx);

function err = lineerr(xy1, xy2)
    y2 = interp1(xy1(:,1), xy1(:,2), xy2(:,1), 'linear');
    idx = ~isnan(y2);
    err = [xy2(idx,1), xy2(idx,2)-y2(idx,1)];
    