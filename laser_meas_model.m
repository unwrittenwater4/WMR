function z_meas=laser_meas_model(z_true,laser_model)

% sample_phit=ones(1,floor(laser_model{1}.p_hit*100));
% sample_short=2*ones(1,floor(laser_model{1}.p_short*100));
% sample_rand=3*ones(1,floor(laser_model{1}.p_rand*100));
% sample_max=4*ones(1,floor(laser_model{1}.p_max*100));
% 
% sample=[sample_phit sample_short sample_rand sample_max];

z_max=laser_model{1}.z_max;
sigma_hit=laser_model{1}.sigma_hit;
lambda=laser_model{1}.lambda_short;
sample=laser_model{1}.sample;
for j=1:length(z_true)
    i=randi(length(sample),1,1);  %generate integer values from uniform set 1:length(sample)
%     i=ceil(length(sample)*rand(1,1));  %generate integer values from uniform set 1:length(sample)
    switch sample(i)
        case 1  % hit
            z_meas(j)=z_true(j)+sqrt(sigma_hit)*randn;
        case 2  % short
            eta=1/(1-exp(-lambda*z_true(j)));
            z_meas(j)=-log(z_true(j)*rand/z_max)/(lambda*eta);            
        case 3  % random
            z_meas(j)=z_max*rand;
        case 4  % max
            z_meas(j)=z_max;
    end
end