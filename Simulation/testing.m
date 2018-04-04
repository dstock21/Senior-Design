angles=zeros(size(input,2),3);
for i=1:size(input,2)
    pos=input(:,i);
    [ang,tf]=ik(pos);
    angles(i,:)=ang';
end 