% map scan

map = imread('mymap.pgm');
map = map<50;
map = double(map);
scan = imread('image1.png');
scan = scan>0;
[i, j] = find(scan>0);

% center scan
[m, n] = size(scan);
centerscan = zeros(m,n);
a1 = max(1, uint8(mean(i))-m/2);
a2 = min(m/2+uint8(mean(i)), m);
b1 = max(1, uint8(mean(j))-n/2);
b2 = min(n/2+uint8(mean(j)), n);
tmp = scan(a1:a2,b1:b2);
[mm, nn] = size(tmp);
centerscan(m/2-floor(mm/2):m/2+floor(mm/2),n/2-floor(nn/2):n/2+floor(nn/2)) = tmp;
%imshow(centerscan)
scan = double(centerscan);



[m, n] = size(map);

rotations = 0:5:360;
section = 50;
scalar = 1;
coloredmap = cell(numel(rotations),1);
prob = zeros(size(map));

for i = 1:numel(rotations)
    rotscan = imrotate(scan,rotations(i));
    res = conv2(double(map),double(rotscan),'same');
    %res = filter2(rotscan,map,'same');
    %res = imfilter(map, rotscan, 'replicate');

%     imagesc(res)
%     drawnow
%     pause(0.05)
    coloredmap{i} = res;
    prob = max(prob,coloredmap{i});
end
prob = prob/max(prob(:));
imagesc(prob)
imagesc((map==0).*prob)

imwrite((map==0).*prob,'probability.png')