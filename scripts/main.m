states = load('toystates.txt');
samples = 2;
obs = zeros(size(states,1), 4);


for ii=1:size(states,1)
states(ii,2) = abs(states(ii,2));
end

newstates = states(:,2:end-1);

scaling = 3;

mu = 0;
sigma = 1 * scaling;

obs = computeObs(newstates, obs, samples, mu,sigma);

flag = write(obs);