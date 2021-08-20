function lmrs(linewidth, linecolor)
% Line Move & Rotate & Scale, key-press function:
%   <-, -> :  for x-axis move
%   ^, v   :  for y-axis move
%   \, /   :  for rotate
%   PgUp, PgDn   :  for scale
%   -, =   :  for change parameter 'dxyra' by /*10
%
% Prototype: lmrs(linewidth, linecolor)
% Inputs: linewidth - specific line width found to Move&Rotate&Scale
%         linecolor - specific line color to be remained before Move&Rotate&Scale
% Output: N/A
%
% Example: myfig, plot(randn(10,1),'linewidth',2); grid on; lmrs;
% 
% See also lmc.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/05/2021
global kpress_para
    kpress_para = [];
    if ~exist('linecolor', 'var'); linecolor='m--';  end
    if ~exist('linewidth', 'var'); linewidth=2.0;  end
    kpress_para.linecolor=linecolor;     kpress_para.linewidth=linewidth; 
    kpress_para.dxyrs=1; 
    kpress_para.dx=0.0; kpress_para.dy=0.0; kpress_para.r=0.0; kpress_para.s=1.0; 
    kpress_para.x=[]; kpress_para.y=[]; 
    hf = gcf;
    title(sprintf('dxyrs=%f; dx=%f, dy=%f, rotate=%f, scale=%f', kpress_para.dxyrs, kpress_para.dx, kpress_para.dy, kpress_para.r, kpress_para.s));
    set(hf,'WindowKeyPressFcn',@keypressfcn)
    
function  keypressfcn(h, evt)
global kpress_para
    switch evt.Key
        case 'leftarrow'
            kpress_para.dx = kpress_para.dx - kpress_para.dxyrs;
        case 'rightarrow'
            kpress_para.dx = kpress_para.dx + kpress_para.dxyrs;
        case 'uparrow'
            kpress_para.dy = kpress_para.dy + kpress_para.dxyrs;
        case 'downarrow'
            kpress_para.dy = kpress_para.dy - kpress_para.dxyrs;
        case 'backslash'  % '\'
            kpress_para.r = kpress_para.r + kpress_para.dxyrs;
        case 'slash'  % '/'
            kpress_para.r = kpress_para.r - kpress_para.dxyrs;
        case 'pageup'
            if kpress_para.s<1.5, kpress_para.s = kpress_para.s + kpress_para.dxyrs/100; end
            if kpress_para.s>1.5, kpress_para.s = 1.5; end
        case 'pagedown'
            if kpress_para.s>0.5, kpress_para.s = kpress_para.s - kpress_para.dxyrs/100; end
            if kpress_para.s<0.5, kpress_para.s = 0.5; end
        case 'equal'
            if kpress_para.dxyrs<1e5, kpress_para.dxyrs = kpress_para.dxyrs*10; end
        case 'hyphen'
            if kpress_para.dxyrs>1e-5, kpress_para.dxyrs = kpress_para.dxyrs/10; end
    end
    title(sprintf('dxyrs=%f, dx=%f, dy=%f, rotate=%f, scale=%f', kpress_para.dxyrs, kpress_para.dx, kpress_para.dy, kpress_para.r, kpress_para.s));
    lines = findall(gca, 'type', 'line');
    for k=1:length(lines)
        hl = lines(k);
        sel = get(hl, 'LineWidth');
        if sel~=kpress_para.linewidth, continue; end
        if isempty(kpress_para.x)
            kpress_para.x = get(hl, 'xdata');    kpress_para.y = get(hl, 'ydata');
            hold on;  plot(kpress_para.x, kpress_para.y, kpress_para.linecolor);
            kpress_para.x0 = kpress_para.x(1); kpress_para.y0 = kpress_para.y(1);
            kpress_para.x = kpress_para.x - kpress_para.x0; kpress_para.y = kpress_para.y - kpress_para.y0; 
        end
        arcmin = pi/180/60;
        x = kpress_para.x*cos(kpress_para.r*arcmin) -  kpress_para.y*sin(kpress_para.r*arcmin);
        y = kpress_para.x*sin(kpress_para.r*arcmin) +  kpress_para.y*cos(kpress_para.r*arcmin);
        x = x*kpress_para.s + (kpress_para.x0 + kpress_para.dx);
        y = y*kpress_para.s + (kpress_para.y0 + kpress_para.dy);
        set(hl, 'xdata', x);      set(hl, 'ydata', y);
        break;
    end
