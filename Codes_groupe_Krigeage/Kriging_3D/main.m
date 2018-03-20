% A partir de mesures d'anomalies magnétiques faite par des robots plus 
% ou moins bien localisés, utiliser une méthode de krigeage pour reconstituer 
% la carte des anomalies magnétiques du fond. 

% Mezher Mohamad 
% Ouadrhiri Mohamed Amine
%%
clear all ; close all ; clc ;

%% 
[X,Y1] = meshgrid(0:299);
Z = randn(size(X));
Z = imfilter(Z,fspecial('gaussian',[40 40],8));
%% Data Loading
load cov.csv
x = cov(1:300,1);
y = cov(1:300,2);
z = cov(1:300,3) ;

%% Ploting the variogram
v = variogram([x y],z,'plotit',false,'maxdist',100);

%% Calculating the range and sill 
[range,sill] = variogramfit(v.distance,v.val,[],[],[],'model','stable');

%% Elevation calculate using the range and sill 
[elevation] = kriging(x,y,z,range,sill);
%
%% Affichage krigeage 3D
figure() , 
surf(X,Y1,elevation) , colorbar , title ('3D');








