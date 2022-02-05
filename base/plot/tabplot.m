function tabplot(varargin)
%  TABPLOT Create axes in tabs.
%  This function will add a tabgroup to the current figure (if not existing)
%  Then it will add a new tab with an axis for plotting (if not existing)
%
%  tabplot % Will create a new tab with axis
%
%  tabplot(title); % Create a new tab with title
%
%  tabplot(index) % Selects a tab from the tabgroup, or create a new tab
%
%  tabplot(index,title)  % Selects a tab from the tabgroup and set title
%
%
%  Example,
%
%  a= linspace(0,2*pi,100);
%  figure,
%  tabplot('Sin'), plot(a,sin(a),'r');
%  tabplot('Tan');
%  tabplot('Cos'), plot(a,cos(a),'g');
%  tabplot(2), plot(a,tan(a),'b');
%
%
%   See also SUBPLOT, GCA, GCF, AXES, FIGURE, UITAB, UITABGROUP
%    
% Written by D.Kroon Dec-2018 at Demcon

h_figure = gcf;
children_figure = findobj(h_figure.Children,'flat','Type','uitabgroup');

title_input = blanks(0);
index = [];
for i=1:nargin
    obj = varargin{i};
    if(ischar(obj))
        title_input = obj;
    elseif(isnumeric(obj))
        index = obj;
    end
end

if(~isempty(children_figure))
    tabgroup = children_figure(1);
else
    tabgroup = uitabgroup(h_figure);
end

children_tabgroup =  findobj(tabgroup.Children,'flat','Type','uitab');

if(isempty(index))
    index = length(children_tabgroup) + 1;
end

if(index > length(children_tabgroup))
    % Open new tab
    for i=length(children_tabgroup)+1:index
        if(i == index && ~isempty(title_input) )
            title = title_input;
        else
            title = ['plot ',num2str(i)];
        end
        tab = uitab(tabgroup,'title',title);
        axes('parent',tab);
    end
else
    tab = children_tabgroup(index);
    if( ~isempty(title_input) )
        tab.Title = title_input;
    end
    children_tab =  findobj(tab.Children,'flat','Type','axes');
    
    % Select axes
    axes(children_tab(1));
end

tabgroup.SelectedTab = tab;


