function FM1_SaveFig(fig,name,descr,in)
file=fullfile(in.out.folder,[name '.png']);
try
    print(fig,'-dpng',sprintf('-r%d',in.out.dpi),file);
catch
    try
        saveas(fig,file);
    catch err
        error('Could not save %s. Check the disk space and write permission of this folder. MATLAB said: %s',file,err.message);
    end
end
fid=fopen(fullfile(in.out.folder,in.out.list_name),'a');
fprintf(fid,'%-28s %s\n',[name '.png'],descr);
fclose(fid);
if ~in.out.keep_open,close(fig); end
end
