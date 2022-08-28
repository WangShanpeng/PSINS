function dgn = dovshow(data)
% Gravity abnomal & Deflection of vertical (DOV) display.
%
% Prototype: dgn = dovshow(data)
% Input: data - 'gndata.txt' data
%
% See also  gdov2inserr, egmwgs84.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/08/2022
global glv
    if nargin<1
        data = load('gndata.txt');
        data(:,2:3) = data(:,[3,2])*glv.deg;
    end
    gn = data(:,end);
    for k=1:length(data)
        eth = earth(data(k,2:4)');
        gn(k) = eth.gn(3);
    end
    dgn = [data(:,end-2:end-1),-(data(:,end)-gn)];
    myfig,
    subplot(221), plot(data(:,2)/glv.deg, dgn/glv.ug), xygo('L / \circ', '\deltag_{E,N,U} / ug'); plot(data(1,2)/glv.deg, dgn(1,:)/glv.ug, '*r')
    subplot(222), plot(data(:,3)/glv.deg, dgn/glv.ug), xygo('\lambda / \circ', '\deltag_{E,N,U} / ug'); plot(data(1,3)/glv.deg, dgn(1,:)/glv.ug, '*r')
    subplot(223), plot(data(:,4), dgn/glv.ug), xygo('H / m', '\deltag_{E,N,U} / ug'); plot(data(1,4), dgn(1,:)/glv.ug, '*r')
    subplot(224), plot(dgn/glv.ug), xygo('k', '\deltag_{E,N,U} / ug')
    