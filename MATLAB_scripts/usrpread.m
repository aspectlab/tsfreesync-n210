% d = usrpread(file, format, complex) read a .dat file from a USRP
% file - file to read
% format - format of data within file
% complex - true/false if data is complex
%
% d = usrpread(file, format) read non complex data
%
% d = usrpread(file, format, complex) read complex data 

function d = usrpread(file, format, complex)
    if nargin < 3
        complex = false;
    else
    end

    fid=fopen(file,'r');
    d=fread(fid,Inf,format);
    fclose(fid);
    
    if complex
        d=d(1:2:end)+1j*d(2:2:end);
    else
    end
end