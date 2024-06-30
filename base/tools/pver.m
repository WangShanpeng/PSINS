function pver()
% PSINS Toolbox Version display
    rpath = psinsenvi();
	disp(['   PSINS Toolbox Version: ',rpath(end-5:end)]);
