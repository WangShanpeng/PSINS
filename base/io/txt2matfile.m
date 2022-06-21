function txt2matfile(fname)
% See also  txtbinfile, matbinfile, txtmatfile.
	files = dir(fname);
    for k=1:length(files)
        disp(sprintf('%s  --- is in processing ...\n', files(k).name));
        data = importdata(files(k).name);
        if isstruct(data)
            data = data.data;
        end
        save([files(k).name(1:end-4),'.mat'], sprintf('data'));
    end
