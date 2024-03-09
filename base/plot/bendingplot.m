function bendingplot(X, X1)
% See also  bending, bendingH, bendingX2KUW.
global glv
    if length(X)==15, X(16:24)=0; end  % KU
    if length(X)==18, x=X; X=zeros(24,1); X([1:6, 7,8,9,11,12,15, 16,17,18,20,21,24])=x; end  % bad
%     if length(X)==21, x=X; X=zeros(24,1); X([1:6, 7:15, 16,17,18,20,21,24])=x; end
    if length(X)==21, X=[0;X(1:2);0;X(3);0;X(4:end)]; end
    if length(X)==21, x=X; X=zeros(24,1); X([1:6, 7:15, 16,17,18,20,21,24])=x; end
    dK = zeros(9,1); dK([1:3,5:6,9]) = X(1:6);
    dK([2:4,6:8]) = dK([2:4,6:8])/glv.sec;  dK([1,5,9]) = dK([1,5,9])/glv.ppm;
    myfig, plot(1:9, dK, '-.o', 11:19, X(7:15)/glv.secpg, '-.o', 21:29, 10*X(16:24)/glv.secprps2, '-.o'); hold on; grid on
    plot([1,5,9], dK([1,5,9]), '*m', [11,15,19], X([7,11,15])/glv.secpg, '*m', [21,25,29], 10*X([16,20,24])/glv.secprps2, '*m');
    legend('dK / ppm,sec', 'U / sec/g', 'W / 0.1sec/(rad/s^2)');
    if nargin==2
        plot(11:19, X1(1:9)/glv.secpg, 'g:*', 21:29, X1(10:18)/glv.secprps2, 'm:*');
    end
    return;
    
    sprintf('[%.2f;%.2f;%.2f]', dK([1,5,9]))
	sprintf('[%.2f;%.2f;%.2f]', dK([2,3,6]))
	sprintf('[%.2f;%.2f;%.2f]\n', X(7:15)/glv.secpg)
	sprintf('[%.2f;%.2f;%.2f]\n', X(16:24)/glv.secprps2)      