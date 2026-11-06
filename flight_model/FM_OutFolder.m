function folder=FM_OutFolder(in)
base=in.out.base_dir;
if isempty(base),base=pwd; end
folder=fullfile(base,in.out.folder);
[ok,msg]=mkdir(folder);
test=fullfile(folder,'FM_write_test.tmp');
fid=-1;
if ok,fid=fopen(test,'w'); end
if fid < 0
    error('Cannot write to the results folder %s. Check in.out.base_dir in FM1_Inputs.m and that the disk is mounted. %s',folder,msg);
end
fclose(fid); delete(test);
fprintf('Results folder: %s\n',folder);
try
    f=java.io.File(folder);
    mb=double(f.getUsableSpace())/1e6;
    fprintf('Free space there: %.0f MB\n',mb);
    if mb < 1000
        fprintf('LOW DISK SPACE: the video may need several hundred MB. Consider setting in.out.base_dir to another disk.\n');
    end
catch
end
end
