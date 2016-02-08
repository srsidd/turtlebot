% map scan

target_hist;

map = imread('mymap.pgm');
[m, n] = size(map);

section = 50;
scalar = 1;
coloredmap = double(map);
for j = 1:n-section
    for i = 1:m-section
        fprintf('%d,%d\n',j,i)
        submap = map(i:i+section,j:j+section);
        imshow(submap)
%         drawnow
%         pause(.05);
        %bwsubmap = submap<50;
%         if i==247
%             merp = 10;
%         end
        % find walls
        [I, J] = find(submap<50);
        dis = scalar*sqrt((I-section/2).^2 + (J-section/2).^2);
        H2 = hist(dis,H(:,2)-.5);
        H2 = reshape(H2,[100,1]);
        D = dist(H(:,2)');
        
        
        [emd]= emd_hat_gd_metric_mex(H(:,1),H2,D,numel(H2)/2,1);
        coloredmap(i+section/2,j+section/2) = emd;
    end
end


