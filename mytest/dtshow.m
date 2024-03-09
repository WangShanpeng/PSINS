function res = dtshow(fres,xyz0,xyz1)
    [fname, m] = dirfile(fres);
    myfig, lgd = {}; nextlinestyle(-1);
    for k=1:m
        res{k} = load(fname{k});
        if ~exist('xyz0','var')
            xyz0 = res{1}(1,1:3)';  xyz1 = res{1}(end,1:3)';
        end
        subplot(121), hold on, plot(res{k}(:,2)-xyz0(2), res{k}(:,1)-xyz0(1), nextlinestyle(1));
        subplot(122), hold on, plot(res{k}(:,4), res{k}(:,3)-xyz0(3), nextlinestyle(0));
%         subplot(122), hold on, plot(res{k}(:,5), res{k}(:,3)-xyz0(3), nextlinestyle(0));
        lgd{k} = fname{k}; % sprintf('%d', k);
    end
	subplot(121), plot([0;xyz1(2)-xyz0(2)],[0;xyz1(1)-xyz0(1)],'om');  xygo('East / m', 'North / m');  legend(lgd);
	subplot(122), plot(res{1}([1,end],4),[0;xyz1(3)-xyz0(3)],'om');  xygo('Distance / m', 'Height / m');
    text(res{1}(end,4)*1.02,(xyz1(3)-xyz0(3)), sprintf('(%.2f; %.2f)',res{1}(end,4),xyz1(3)-xyz0(3)));
    if exist('maxlh.txt', 'file')
        maxlh = load('maxlh.txt');
        subplot(121), title(sprintf('水平弥散=%.3fm',maxlh(1)));
        idx = find(res{end}(:,4)>maxlh(2),1);
        if ~isempty(idx)
            plot(res{end}(idx,2)-xyz0(2),res{end}(idx,1)-xyz0(1),'*r','linewidth',3);
        end
        subplot(122), title(sprintf('高程弥散=%.3fm',maxlh(3)));
        idx = find(res{end}(:,4)>maxlh(4),1);
        if ~isempty(idx)
            plot(res{end}(idx,4),res{end}(idx,3)-xyz0(3),'*r','linewidth',3);
        end
    end
    