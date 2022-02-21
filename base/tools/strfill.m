function str = strfill(substr, len, char_end)
    if nargin<3, char_end=0; end
    if len<=length(substr), substr=substr(1:len-1); end
    str = substr;
    str(length(substr)+1:len) = char(char_end);
%     n = ceil(len/length(substr));
%     str = repmat(substr,1,n);
%     str(len+1:end) = [];
%     if nargin>2, str(end)=char(char_end); end