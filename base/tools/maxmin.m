function [vvmax, vvmin] = maxmin(val)
persistent vmax vmin per_vmax per_vmin per_maxcnt
    if isempty(vmax)
        vmax = val; vmin = val;
        per_vmax = val; per_vmin = val;
        per_maxcnt = 10;
    end
    if per_vmax<val, per_vmax = val; end
    if per_vmin>val, per_vmin = val; end
    per_maxcnt = per_maxcnt - 1;
    if per_maxcnt<=0, vmax = per_vmax; vmin = per_vmin; per_vmax = val; per_vmin = val; per_maxcnt=300; end
    vvmax = vmax;
    vvmin = vmin;