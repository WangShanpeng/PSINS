function satplot(PRN, AzEl, mycontour)
% Plot satellite position on the polar sky.
%
% See also  obsplot.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/08/2015
    if nargin<3, mycontour=[15;30;45;60;75]; end
    mycontour = [0;mycontour]*pi/180;
    afa=(0:0.1:360)*pi/180;
    myfigure, subplot(1,5,2:4);
    for k=1:length(mycontour)
        x = cos(mycontour(k))*sin(afa); y = cos(mycontour(k))*cos(afa);
        plot(x,y,':b'); hold on;
        if k>1, text(max(x), 0, [num2str(mycontour(k)*180/pi),'\circ']); end
    end
    a = sqrt(2)/2;
    plot([-1,1],[0,0],':b', [0,0],[-1,1],':b', [-a,a],[-a,a],':b', [-a,a],[a,-a],':b');
    axis equal; xlabel('West-East'); ylabel('South-North');
    clr = 'bgrcmyk';  clr = repmat(clr,1,10);    
    PRNu = unique(PRN);
    for k=1:length(PRNu)
        AzEli = AzEl(PRN==PRNu(k),:);
        x = cos(AzEli(:,2)).*sin(AzEli(:,1)); y = cos(AzEli(:,2)).*cos(AzEli(:,1));
        plot(x,y,['-x',clr(k)]); 
        text(x(1), y(1), num2str(PRNu(k)));
    end

