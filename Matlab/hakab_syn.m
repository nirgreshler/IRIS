num_exp = 100;
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

while exp_idx <= num_exp
    try
        createSimpleEnv;
        
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
        copyfile([original_graph_path '_vertex'], ['Results\' env_name '_vertex_' num2str(exp_idx)]);
        copyfile([original_graph_path '_conf'], ['Results\' env_name '_conf_' num2str(exp_idx)]);
        copyfile([original_graph_path '_edge'], ['Results\' env_name '_edge_' num2str(exp_idx)]);
        copyfile([original_graph_path '_inspectionPoints'], ['Results\' env_name '_inspectionPoints_' num2str(exp_idx)]);
        copyfile([original_graph_path '_obstacles'], ['Results\' env_name '_obstacles_' num2str(exp_idx)]);
        copyfile([original_graph_path '_params'], ['Results\' env_name '_params_' num2str(exp_idx)]);
        
        res = table(runtime_original, build_bridge_time, runtime_bridge, ...
            cov, cov_bridge, cost_orig, cost_bridge, num_clusters,...
            nv_orig, ne_orig, nv_bridge, ne_bridge);
        
        save(['Results\' env_name '_results'], 'res');
        
        disp(mean(res.cov(1:exp_idx))/400);
        disp(mean(res.cov_bridge(1:exp_idx))/400);
        pause(0.5);
        exp_idx = exp_idx + 1;
    catch
    end
end

% avg = mean(res.Variables);
% res12 = res(res.num_clusters > 12, :);
% avg_big_clusters12 = mean(res12.Variables);
% 
% res15 = res(res.num_clusters > 15, :);
% avg_big_clusters15 = mean(res15.Variables);

nClusters = unique(res.num_clusters);
m_cov = zeros(1, length(nClusters));
m_bridge_cov = zeros(1, length(nClusters));
m_cost = zeros(1, length(nClusters));
m_bridge_cost = zeros(1, length(nClusters));
m_runtime = zeros(1, length(nClusters));
m_bridge_runtime = zeros(1, length(nClusters));
for i = 1:length(nClusters)
    resC = res(res.num_clusters == nClusters(i), :);
    avg_cluster = mean(resC.Variables);
    m_cov(i) = mean(resC.cov) / 400;
    m_bridge_cov(i) = mean(resC.cov_bridge) / 400;
    m_cost(i) = mean(resC.cost_orig);
    m_bridge_cost(i) = mean(resC.cost_bridge);
    m_runtime(i) = mean(resC.runtime_original);
    m_bridge_runtime(i) = mean(resC.runtime_bridge);
end

close all;

plot(nClusters, m_cov, 'Marker', 'o', 'LineWidth', 2, 'DisplayName', 'Original Coverage');
hold on;
plot(nClusters, m_bridge_cov, 'Marker', 'x', 'LineWidth', 2, 'DisplayName', 'Bridge Coverage');
grid;
legend('show');
xlabel('# of clusters');
ylabel('% coverage');
% 
% figure;
% plot(nClusters, m_cost, 'Marker', 'o', 'LineWidth', 2, 'DisplayName', 'Original Cost');
% hold on;
% plot(nClusters, m_bridge_cost, 'Marker', 'x', 'LineWidth', 2, 'DisplayName', 'Bridge Cost');
% grid;
% legend('show');
% xlabel('# of clusters');
% ylabel('path cost');
% 
% figure;
% semilogy(nClusters, m_runtime, 'Marker', 'o', 'LineWidth', 2, 'DisplayName', 'Original Runtime');
% hold on;
% semilogy(nClusters, m_bridge_runtime, 'Marker', 'x', 'LineWidth', 2, 'DisplayName', 'Bridge Runtime');
% grid;
% legend('show');
% xlabel('# of clusters');
% ylabel('log(runtime)');

set(findall(gcf,'-property','FontSize'),'FontSize',14)
title('Coverage per # of Clusters for 9 Rooms')

%% Analyze
minNumClustersToPlot = 0;
resToPlot = res(1:find(res.runtime_original ~=0, 1, 'last'),:);
resToPlot = resToPlot(resToPlot.num_clusters > minNumClustersToPlot, :);
figure;
hSp(1) = subplot(311); hold on
plot(100*resToPlot.cov_bridge./resToPlot.cov, '.-')
med = 100*median(resToPlot.cov_bridge./resToPlot.cov);
plot([1 size(resToPlot.cov,1)], [1 1].*med, '--', 'LineWidth', 3)
title('Bridge Coverage / Original Coverage')
xlabel('Experiment #'); ylabel('Coverage ratio (%)');
ylim([0 120])
grid;
legend('Ratio for each Experiment', sprintf('Median over Experiments (%.2f%%)', med), 'Location', 'Best')

hSp(2) = subplot(312); hold on
plot(100*resToPlot.runtime_bridge./resToPlot.runtime_original, '.-')
med = 100*median(resToPlot.runtime_bridge./resToPlot.runtime_original);
plot([1 size(resToPlot.cov,1)], [1 1].*med, '--', 'LineWidth', 3)
title('Bridge Runtime / Original Runtime')
xlabel('Experiment #'); ylabel('Runtime ratio (%)');
ylim([0 120])
grid;
legend('Ratio for each Experiment', sprintf('Median over Experiments (%.2f%%)', med), 'Location', 'Best')

hSp(3) = subplot(313); hold on
plot(100*resToPlot.cost_bridge./resToPlot.cost_orig, '.-')
med = 100*median(resToPlot.cost_bridge./resToPlot.cost_orig);
plot([1 size(resToPlot.cov,1)], [1 1].*med, '--', 'LineWidth', 3)
title('Bridge Cost / Original Cost')
xlabel('Experiment #'); ylabel('Cost ratio (%)');
ylim([0 120])
grid;
legend('Ratio for each Experiment', sprintf('Median over Experiments (%.2f%%)', med), 'Location', 'Best')

set(findall(gcf,'-property','FontSize'),'FontSize',14)

if minNumClustersToPlot > 0
    sgtitle(sprintf('Results for 9 Rooms (>%d clusters)', minNumClustersToPlot))
else
    sgtitle('Results for 9 Rooms')
end
linkaxes(hSp, 'x')