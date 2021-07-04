function I = ImageAverage(varargin)

% ===============================================================================================
% This function is used to fuse a sequences of images with the same size;
% The intensity on overlapping areas is the mean of them
% Version:1.0
% ===============================================================================================
% 
% Copyright (C) 2021 Zhehua Mao
% University of Technology, Sydney, Australia
% 
% Authors:  Zhehua Mao        -- zhehua.mao@student.uts.edu.au
% 
%           Centre for Autonomous Systems
%           Faculty of Engineering and Information Technology
%           University of Technology, Sydney
%           NSW 2007, Australia
% 
% Please contact Zhehua Mao {zhehua.mao@student.uts.edu.au} if you have any questions/comments about the code.


weight = zeros(size(varargin{1}));
I_sum  = zeros(size(varargin{1}));
for numFrames = 1:nargin
    weight = weight + logical(varargin{numFrames}); 
    I_sum = I_sum + varargin{numFrames};
end
weight = 1./weight;
weight(weight==inf)=0;
I = weight.*I_sum;
end