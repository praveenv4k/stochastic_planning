function [ obs ] = computeObs( states,obs, samples, mu, sigma )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

totY = unique(states(:,2));

for ii=1:size(states)
%for each state, find #samples nearest neighbours
    
    [index, D] = knnsearch(totY,states(ii,2),'K',samples,'distance', 'euclidean');

   
    temp = zeros(samples,2);
   %assign probabilities to nn
   for jj=1:samples
      
       pr = mygaussian(0 + D(1,jj)/10, mu, sigma);
       
        temp(jj,1) = index(1,jj);
        temp(jj,2) = pr;
     
   end
 
%write new line in obs matrix
for kk=1:samples
    obs(ii,temp(kk,1)) = temp(kk,2); 
end

end
 
obs = normalize(obs);

for bb=1:size(obs,1)
    tot = sum(obs(bb,:));
    [c, index] = max(obs(bb,:));
    obs(bb,index) = c + 1-tot;
    
end



end

