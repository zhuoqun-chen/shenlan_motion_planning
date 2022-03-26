function obstable_map = obstable_map(MAP, MAX_X, MAX_Y)
%Put all obstacles on the Closed list
k=1;%Dummy counter
obstable_map = [];
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            obstable_map(k,1)=i;
            obstable_map(k,2)=j;
            k=k+1;
        end
    end
end
end