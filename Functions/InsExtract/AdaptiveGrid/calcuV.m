function Ve = calcuV(points)
%CALCUV 
% centralize points
% drowPts(points,'.r')
if size(points,1) * size(points,2) == 0
    Ve = 0;
    return
end
mean_points = mean(points);
points_centered = bsxfun(@minus, points, mean_points);

% Calculate covariance matrix
cov_matrix = cov(points_centered);

% Calculate the eigenvalues and eigenvectors of the covariance matrix
[V,D] = eig(cov_matrix);

% Sort according to the eigenvalues and find the corresponding eigenvectors
[D_sort, D_order] = sort(diag(D), 'descend');
V_sort = V(:,D_order);

% Find the main eigenvector, which is the direction vector
dir_vector = V_sort(:,1);

% Normalize direction vectors
length = norm(dir_vector);
norm_vector = dir_vector / length;

% Calculate verticality
Ve = abs(norm_vector(3));
end

