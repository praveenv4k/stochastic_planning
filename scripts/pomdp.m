%leggere file con stati


for ii=1:size(states,1)
states(ii,2) = abs(states(ii,2));
end

for ii=1:size(states)
%per ogni stato, trovare tot nearest neighbour
    
    [index, D] = knnsearch(states,states(ii,:),'K',samples,'distance', 'euclidean');

   
    temp = zeros(samples+1,2);
   %assegnare probabilità ai neigh 
   for jj=1:samples
      
       pr = mygaussian(0 + D(1,jj)/10);
       
        temp(jj,1) = index(1,jj);
        temp(jj,2) = pr;
     
   end
 
%scrivere riga di obs matrix
for kk=1:samples
    obs(ii,temp(kk,1)) = temp(kk,2); 
end

%normalize prob
tot = sum(obs(1,:));


end

for bb=1:240
     for hh = 1:240
         temp = obs(bb,hh)/tot;
         obs(bb,hh) = temp;
     end
 end