% function [ i ] = write(obs )
% fid = fopen('observation.txt', 'w+');
% 
% for ii=1:240
%     for jj = 1:240
%     mystr = strcat('O:' , num2str(ii) , ':' , num2str(jj) , ':' , num2str(obs(ii,jj)),'\n');
%     fprintf(fid,mystr);
%     end
% end
% i = 1;
% end

function [ i ] = write(obs )
fid = fopen('toy_reduced_observation.txt', 'w+');
mystr = 'O: * \n';
fprintf(fid,mystr);
for ii=1:size(obs,1)
    for jj = 1:size(obs,2)
   % mystr = strcat(num2str(obs(ii,jj)));
    mystr = num2str(obs(ii,jj));
    fprintf(fid,'%s ',mystr);
    end
    mystr = '\n';
 fprintf(fid,mystr);
end
 
i = 1;
end