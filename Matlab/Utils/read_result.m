function pathIdx = read_result(res_file)

fid = fopen(res_file);
C = textscan(fid, '%s', 'delimiter','\n');
fclose(fid);
C = C{1, 1};
out = C{end};
outsplt = strsplit(strtrim(out), ' ');
pathIdx = str2double(outsplt(2:end));