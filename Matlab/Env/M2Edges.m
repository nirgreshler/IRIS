function edges = M2Edges(M)
edges = NaN(numel(M),3);
counter = 1;
for k = 1:size(M,2)
    for p = k+1:size(M,2)
        if M(k,p) > 0
            edges(counter,:) = [k-1 p-1 1/M(k,p)];
            counter = counter+1;
        end
    end
end
vIdcs = ~isnan(edges(:,1));
edges = edges(vIdcs,:);