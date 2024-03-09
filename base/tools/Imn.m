function data = Imn(data)
% See also  Omn
    if length(data)==1
        data = ones(data,1);
    end
    data = ones(size(data));