function [ normMatrix ] = normalize( matrix )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


for bb=1:size(matrix,1)
    tot = sum(matrix(bb,:));
     for hh = 1:size(matrix,2)
         temp = matrix(bb,hh)/tot;
         matrix(bb,hh) = temp;
     end
end

matrix = single(matrix);
matrix = chop(matrix,3);
 normMatrix = matrix;

end

