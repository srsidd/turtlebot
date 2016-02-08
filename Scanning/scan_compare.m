
% im = scan2im(s(1,:));
% %subplot(1,2,1),imagesc(im)
% 
% im2 = scan2im(s(2,:));
% %subplot(1,2,2),imagesc(im2)
% %imshowpair(im,im2,'montage')

im = imread('image1.png');
for i = 1:643
    imname = ['im' num2str(i) '.png'];
    im2 = imread(imname);
    tformEstimate = imregcorr(im2,im);
    Rfixed = imref2d(size(im));
    im3 = imwarp(im2,tformEstimate,'OutputView',Rfixed);
    
    imname2 = ['corr' num2str(i) 'b.png'];
    imwrite(im3, imname2);
end
