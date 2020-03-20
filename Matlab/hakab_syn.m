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
    res = table(runtime_original, build_bridge_time, runtime_bridge, ...
        cov, cov_bridge, cost_orig, cost_bridge);
    first_exp = 1;
else
    first_exp = exp_idx;
end

for exp_idx = first_exp:num_exp
    createSimpleEnv;
    
    run_search;
    
    runtime_original(exp_idx) = runtime_original_;
    build_bridge_time(exp_idx) = build_bridge_time_;
    runtime_bridge(exp_idx) = runtime_bridge_;
    cov(exp_idx) = length(cov_set_);
    cov_bridge(exp_idx) = length(cov_set_bridge_);
    cost_orig(exp_idx) = cost_orig_;
    cost_bridge(exp_idx) = cost_bridge_;
    
    f = gcf;
    savefig(f, ['Results\' env_name '_' num2str(exp_idx)]);
    copyfile([original_graph_path '_vertex'], ['Results\' env_name '_vertex_' num2str(exp_idx)]);
    copyfile([original_graph_path '_conf'], ['Results\' env_name '_conf_' num2str(exp_idx)]);
    copyfile([original_graph_path '_edge'], ['Results\' env_name '_edge_' num2str(exp_idx)]);
    copyfile([original_graph_path '_inspectionPoints'], ['Results\' env_name '_inspectionPoints_' num2str(exp_idx)]);
    copyfile([original_graph_path '_obstacles'], ['Results\' env_name '_obstacles_' num2str(exp_idx)]);
    copyfile([original_graph_path '_params'], ['Results\' env_name '_params_' num2str(exp_idx)]);
    
    res = table(runtime_original, build_bridge_time, runtime_bridge, ...
        cov, cov_bridge, cost_orig, cost_bridge);
    
    save(['Results\' env_name '_results'], 'res');
    
    disp(mean(res.cov(1:exp_idx))/400);
    disp(mean(res.cov_bridge(1:exp_idx))/400);
    pause(0.5);
end

