% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
% thre = 
% mu =    [151.4004  145.3483   58.2015]';
% sig =   [ 149.4123   98.5156 -152.8467; ...
%           98.5156  123.3045 -159.0817; ...
%           -152.8467 -159.0817  279.0633];
mu = [0.1559    0.6097    0.5967]';
sig =     [0.0003   -0.0000   -0.0002; ...
   -0.0000    0.0172    0.0050; ...
   -0.0002    0.0050    0.0020];

I = rgb2hsv(I);
thre = 0.3;
invSig = inv(sig);      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
bw = false([size(I, 1), size(I, 2)]);
bp = zeros([size(I, 1), size(I, 2)]);
for row = 1:size(I, 1)
    for col = 1:size(I, 2)
        pixel = double(reshape(I(row, col, :), [3 1]));
%         p = mvnpdf(double(pixel), mu, sig);
        p = exp(-0.5 * (pixel - mu)' * invSig * (pixel - mu));
        bp(row, col) = p;
        if (p>thre) 
            bw(row, col) = true;
        end
    end
end   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
% create new empty binary image
bw_biggest = false(size(bw));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
S = regionprops(CC,'Centroid');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)
segI = bw_biggest;
loc = S(idx).Centroid;
end
