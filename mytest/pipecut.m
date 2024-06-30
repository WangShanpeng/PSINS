function posres = pipecut(pos, pos1, pos2)
    if nargin<3, pos2=pos1(end,1:3)'; pos1=pos1(1,1:3)'; end
    cL = cos(pos1(1));
    dis = norm([pos2(1)-pos1(1); (pos2(2)-pos1(2))*cL]);
    d1 = norm([pos(end,1)-pos1(1); (pos(end,2)-pos1(2))*cL]);
    d2 = norm([pos(end,1)-pos2(1); (pos(end,2)-pos2(2))*cL]);
    if d1<d2, p=pos2; pos1=pos2; pos2=p; end
    len = length(pos);
    idx2 = len;
    for k=len:-1:len/2
        d1 = norm([pos(k-1,1)-pos1(1); (pos(k-1,2)-pos1(2))*cL]);
        d2 = norm([pos(k,1)-pos1(1); (pos(k,2)-pos1(2))*cL]);
        if (d1-dis)*(d2-dis)<=0
            if d1<=dis,  pos(k,:) = pos(k-1,:) + (dis-d1)/(d2-d1)*(pos(k,:)-pos(k-1,:));
            else,        pos(k,:) = pos(k,:)   + (dis-d2)/(d1-d2)*(pos(k-1,:)-pos(k,:));    end
            idx2 = k;   break;
        end
    end
	idx1 = 1;
    for k=1:len/2
        d1 = norm([pos(k,1)-pos2(1); (pos(k,2)-pos2(2))*cL]);
        d2 = norm([pos(k+1,1)-pos2(1); (pos(k+1,2)-pos2(2))*cL]);
        if (d1-dis)*(d2-dis)<=0
            if d1>=dis,   pos(k,:) = pos(k,:)   + (dis-d1)/(d2-d1)*(pos(k+1,:)-pos(k,:));
            else,         pos(k,:) = pos(k+1,:) + (dis-d2)/(d1-d2)*(pos(k,:)-pos(k+1,:));   end
            idx1 = k;   break;
        end
    end
    posres = pos(idx1:idx2,:);
%     pos2dplot(pos, posres+0.01/6378137, [[pos1',1];[pos2',2]]);
    