function cep = rms2cep(rmsv, cepn)
% Translate RMS to CEP.
%
% Prototype: cep = rms2cep(rmsv, cepn)
% Inputs: rmsv - RMS data or raw data
%         cepn - = 50, 90 or 95
% Output: CEP values
%
% Examples:
%    rms2cep([1,2, 0.5], 50);
%    x=randn(10,1)+2; y=3*randn(10,1); rms2cep([x,y], 50);
%
% See also  cep, rmsupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/06/2022
    if nargin<2, cepn=50; end
    [m,n] = size(rmsv);
    mn = m*n;
    if mn==1, rmsv = [rmsv(1),0; 0 rmsv(1)];  % [rms1,rms1]
    elseif mn==2, rmsv = [rmsv(1),0; 0 rmsv(2)];  % [rms1,rms2]
    elseif mn==3, rmsv = [rmsv(1),rmsv(3); rmsv(3),rmsv(2)];  % [rms1,rms2,rho]
    elseif mn==4, rmsv = rmsv;  % 2x2 matrix [rms1,rho; rho rms2]
    elseif mn>4, % [x,y] data
        c = corrcoef(rmsv(:,1), rmsv(:,2));
        cep = rms2cep([rms(rmsv(:,1)),rms(rmsv(:,2)),c(1,2)], cepn);
        plot(rmsv(:,1),rmsv(:,2),'*', 0,0,'o');  m=max(max(max(abs(rmsv))),cep(end)); xlim([-m,m]); ylim([-m,m]);
        return;
    end
    sP = [rmsv(1,1)^2,rmsv(1,1)*rmsv(2,2)*rmsv(1,2); rmsv(1,1)*rmsv(2,2)*rmsv(1,2), rmsv(2,2)^2];
    pp = sP^-1;
    ss = sprintf('%.2f*x^2 %+.2f*x*y + %.2f*y^2 - 1', pp(1,1), 2*pp(1,2), pp(2,2));
    myfigure;
    ezplot(ss); hold on; grid on; axis equal;  m=max(rmsv(1,1),rmsv(2,2))*1.2; xlim([-m,m]); ylim([-m,m]);
    len = 100000;
    x = rmsv(1,1)*randn(len,1); y = rmsv(2,2)*randn(len,1);
    r = sort(sqrt(x.^2 + y.^2));  cep = r(fix(len*[0.5,0.9,0.95]));
    r = r(fix(len*cepn/100));  p=r^-2;
    ss = sprintf('%.2f*x^2 %+.2f*y^2 - 1', p, p);
    ezplot(ss); title(sprintf('CEP%d=%.2f (%.2f)', cepn, r, 0.5887*(rmsv(1,1)+rmsv(2,2))));
    

