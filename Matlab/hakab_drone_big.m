if ~exist('exp_idx', 'var')
    clear;
    num_exp = 100;
    runtime_original = zeros(num_exp, 1);
    build_bridge_time = zeros(num_exp, 1);
    runtime_bridge = zeros(num_exp, 1);
    cov = zeros(num_exp, 1);
    cov_bridge = zeros(num_exp, 1);
    cost_orig = zeros(num_exp, 1);
    cost_bridge = zeros(num_exp, 1);
    num_clusters = zeros(num_exp, 1);
    nv_orig = zeros(num_exp, 1);
    ne_orig = zeros(num_exp, 1);
    nv_bridge = zeros(num_exp, 1);
    ne_bridge = zeros(num_exp, 1);
    res = table(runtime_original, build_bridge_time, runtime_bridge, ...
        cov, cov_bridge, cost_orig, cost_bridge, num_clusters, ...
        nv_orig, ne_orig, nv_bridge, ne_bridge);
    exp_idx = 1;
end

define_path;

filename = 'drone_big_2000';
file_to_read = [base_name_in_wsl '/' filename];

while exp_idx <= num_exp
    try
        
        build_path = [wsl_path, '/build_drone_big/app/build_graph'];
        % build a graph
        system(['wsl ' build_path ' ' num2str(exp_idx) ' 2000 ' file_to_read]);
        
        run_search;
        
        runtime_original(exp_idx) = runtime_original_;
        build_bridge_time(exp_idx) = build_bridge_time_;
        runtime_bridge(exp_idx) = runtime_bridge_;
        cov(exp_idx) = length(cov_set_);
        cov_bridge(exp_idx) = length(cov_set_bridge_);
        cost_orig(exp_idx) = cost_orig_;
        cost_bridge(exp_idx) = cost_bridge_;
        num_clusters(exp_idx) = nClusters;
        nv_orig(exp_idx) = G.num_vertices();
        ne_orig(exp_idx) = G.graph.numedges();
        nv_bridge(exp_idx) = BG.num_vertices();
        ne_bridge(exp_idx) = BG.graph.numedges();
        
        f = gcf;
        savefig(f, ['Results\' env_name '_' num2str(exp_idx)]);
%         copyfile([original_graph_path '_vertex'], ['Results\' env_name '_vertex_' num2str(exp_idx)]);
%         copyfile([original_graph_path '_conf'], ['Results\' env_name '_conf_' num2str(exp_idx)]);
%         copyfile([original_graph_path '_edge'], ['Results\' env_name '_edge_' num2str(exp_idx)]);
%         copyfile([original_graph_path '_inspectionPoints'], ['Results\' env_name '_inspectionPoints_' num2str(exp_idx)]);
%         copyfile([original_graph_path '_obstacles'], ['Results\' env_name '_obstacles_' num2str(exp_idx)]);
%         copyfile([original_graph_path '_params'], ['Results\' env_name '_params_' num2str(exp_idx)]);
%         
        res = table(runtime_original, build_bridge_time, runtime_bridge, ...
            cov, cov_bridge, cost_orig, cost_bridge, num_clusters,...
            nv_orig, ne_orig, nv_bridge, ne_bridge);
        
        save(['Results\' env_name '_results'], 'res');
        
        disp(mean(res.cov(1:exp_idx))/14021);
        disp(mean(res.cov_bridge(1:exp_idx))/14021);
%         pause(0.5);
        exp_idx = exp_idx + 1;
    catch
        disp('error');
    end
end

avg = mean(res.Variables);
res15 = res(res.num_clusters > 15, :);
avg_big_clusters15 = mean(res12.Variables);

res20 = res(res.num_clusters > 20, :);
avg_big_clusters20 = mean(res15.Variables);