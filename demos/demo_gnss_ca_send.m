% GNSS CA send demostration, from star to receiver.
% See also  NA.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/11/2021
function demo_gnss_ca_send
    [ca1, idx1, k1max] = genca([1 1 1 0 0 1 0 0]);
    [ca2, idx2, k2max] = genca([1 1 1 0 1 0 0 0]);
    hfig = myfig;
    k1=k1max;
    k2=k2max;
    while 1
        if ~ishandle(hfig),  break;  end
        k1 = preidx(idx1, k1, k1max);   k2 = preidx(idx2, k2, k2max);
        hold off;
        x1 = -40; y1 = 40;
        x2 = 40; y2 = 40;
        plot(x1, y1, 'pb', 'linewidth', 5); hold on
        plot(x2, y2, 'pm', 'linewidth', 5);
        plot(1, 0, 'vr', 'linewidth', 5);
        plot(-10, -12, 'vg', 'linewidth', 5);
        caplot(idx1, ca1, -60*pi/180, k1, x1, y1, 'b');
        caplot(idx1, ca1, -45*pi/180, k1, x1, y1, 'b');
        caplot(idx2, ca2, -135*pi/180, k2, x2, y2, 'm');
        legend('Star-1', 'Star-2', 'Receiver-1', 'Receiver-2');
        xlim([-50,70]), ylim([-50,50]);
        pause(0.5);
    end
    
function [ca, idx, kca0] = genca(ca0)
    n = length(ca0);
    ca = ca0(1); k1 = 2; idx(1) = 1;
    for k=2:n
        if ca0(k)~=ca0(k-1)
            ca(k1) = ca0(k-1); idx(k1)=k; k1=k1+1;
        end
        ca(k1) = ca0(k); idx(k1)=k; k1=k1+1;            
    end
    kca0 = length(ca);
    ca = repmat(ca,1,10);  idx = repidx(idx,10);
    
function idx1 = repidx(idx, k)
    idx1 = idx;
    for kk=2:k
        idx1 = [idx1, idx+(kk-1)*idx(end-1)];
    end

function kp = preidx(idx, k, maxk)
    kp = k-1;  if kp<1, kp = maxk; end
    if idx(kp)==idx(k), kp = kp-1; end
    if kp<1, kp = maxk; end

function caplot(idx, ca, afa, sft, x0, y0, clr)
	ca = ca(sft:end); idx = idx(sft:end)-idx(sft)+1;  idx = idx*2;
	plot(x0+idx*cos(afa)-ca*sin(afa), y0+idx*sin(afa)+ca*cos(afa), clr);
%     idx = (10-sft):11:length(ca);  ca=ones(size(idx))/2
% 	plot(x0+idx*cos(afa)-ca*sin(afa), y0+idx*sin(afa)+ca*cos(afa), [clr,'.'], 'linewidth', 5);
   


