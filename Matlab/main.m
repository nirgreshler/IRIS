K_MEANS_START = 1;
K_MEANS_END = 20;
k_vec = K_MEANS_START:K_MEANS_END;
k_vec = 3:10;
CVG_TH = 0.2;
dist = 'sqeuclidean';
base_name = 'C:\Users\nirgreshler\GitRepos\IRIS\Matlab\Graphs\syn_4rooms';

[conf, vertex, edges] = read_graph(base_name);

% Show the graph
% s = edges(:, 1) + 1;
% t = edges(:, 2) + 1;
% G = graph(s,t);
% figure;
% h = plot(G);

% Define distance metric for configuration
conf_dist = @(c1, c2) norm(c1 - c2);

% Define the data for clustering (configuration data)
conf_data = conf(:, 2:end);

all_inp_pts = vertex(:, 4:end);
all_inp_pts = all_inp_pts(:);
all_inp_pts = all_inp_pts(~isnan(all_inp_pts));
total_inspection_points = max(all_inp_pts); % TODO

% perform k-means
c = colormap('hsv');
close(gcf);
rng(1);
c = c(randperm(size(c, 1)), :);

diff_vec = zeros(1, length(k_vec));
for k_idx = 1:length(k_vec)
    k = k_vec(k_idx);
    [idx,C] = kmeans(conf_data, k, 'Display', 'off', 'Distance', dist);
    figure; 
    scatter(conf_data(:, 1), conf_data(:, 2), 5, c(idx, :));
    title(['K=' num2str(k)]);
    
    % Check which coverage we have in each cluster
    cov_set_per_cluster = cell(k, 1);
    figure; 
    for i = 1:k
        cl_vertex_idx = idx == i;
        cl_vertex_cov = vertex(cl_vertex_idx, 4:end);
        cl_vertex_cov_col = cl_vertex_cov(:);
        cl_vertex_cov_col = cl_vertex_cov_col(~isnan(cl_vertex_cov_col));
%         figure; histogram(cl_vertex_cov_col, unique(cl_vertex_cov_col))
        [upts, ia, ic] = unique(cl_vertex_cov_col);
        num_inspection_points = length(upts);
        N = arrayfun(@(x) sum(cl_vertex_cov_col == x), upts);
%         h = histcounts(cl_vertex_cov_col, unique(cl_vertex_cov_col)); 
%         N = h.BinCounts;
        maxN = max(N);
        distb = N ./ maxN;
        
        cov_set_per_cluster{i, 1} = upts(distb > CVG_TH);
        
        plot(upts, N, 'DisplayName', ['Cluster ' num2str(i)], 'Color', c(i, :), 'LineStyle', 'None', 'Marker', 'o'); 
        hold on;
        
    end
    title(['K=' num2str(k)]);
%     legend('show');
    sim_mat = ones(k, k)*nan;
    
    for i = 1:k
        for j = i+1:k
            ints = intersect(cov_set_per_cluster{i}, cov_set_per_cluster{j});
            unio = union(cov_set_per_cluster{i}, cov_set_per_cluster{j});
            jaccard_dist = length(ints) / length(unio);
            sim_mat(i, j) = jaccard_dist;
        end
    end
    
    sim_values = sim_mat(~isnan(sim_mat));
    diff_measure = mean(sim_values);
    diff_vec(k_idx) = diff_measure;
    %disp(['K=' num2str(k) ', Diff=' num2str(diff_measure)]);
end

figure;
plot(k_vec, diff_vec);
