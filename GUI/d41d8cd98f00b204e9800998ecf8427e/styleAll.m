%Apply styling to each figure in the current directory

al=dir('*.fig');

for i=1:length(al)
    h=hgload(al(i).name)
    stylefig(h,'default.css');
    hgsave(h,al(i).name);
    close(h);
end