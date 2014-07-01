vec = zeros(404,1);

for bb=1:size(obs,1)
    tot = sum(obs(bb,:));
    vec(bb,1) = tot;
end
 