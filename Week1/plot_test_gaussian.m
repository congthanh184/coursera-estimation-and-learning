lay1 = 1; lay2 = 3;

scatter(Samples(:, lay1), Samples(:, lay2));
hold on;

mu = mu_SG([lay1 lay2]);
Sigma = sigma_SG([lay1 lay2], [lay1 lay2]);
% Sigma = diag(sigma_SG(1:2));
min_lay1 = double(min(Samples(:, lay1)));
max_lay1 = double(max(Samples(:, lay1)));
min_lay2 = double(min(Samples(:, lay2)));
max_lay2 = double(max(Samples(:, lay2)));

x1 = linspace(min_lay1, max_lay1, 500); 
x2 = linspace(min_lay2, max_lay2, 500);
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));


contour(x1,x2,F);