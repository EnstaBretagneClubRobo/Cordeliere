function init
close all; clear all; clc;
W = get(0,'ScreenSize'); % plein ecran
figure('Position',[0 0 W(3) W(4)]);
set(gca,'FontSize',12);
%axis[xmin xmax ymin ymax];
axis square

%f = getframe;              %Capture screen shot
%[im,map] = frame2im(f);    %Return associated image data
%imwrite(im,'image.bmp');
end

