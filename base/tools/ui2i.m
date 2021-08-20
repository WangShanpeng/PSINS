function ii = ui2i(ui, fmt)
% trans 1/2/3/4*uint8 array to int8/16/32/64
    [NA, m] = size(ui);
    if ~exist('fmt','var')
        if m==1, fmt = [1];
        elseif m==2, fmt = [1,2];
        elseif m==4, fmt = [3,4,1,2];
        elseif m==8, fmt = [7,8,5,6,3,4,1,2];
        end
    end
    ii = ui(:,fmt(1));
    for k=2:m, ii = ii + ui(:,fmt(k))*256^(k-1);  end
    idx = ui(:,fmt(end))>127;
    ii(idx) = ii(idx)-2^(8*m);