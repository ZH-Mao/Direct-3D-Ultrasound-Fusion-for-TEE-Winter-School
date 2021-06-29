function I2 =Registration(I1, T)
% ===============================================================================================
% Register I1 with T
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

I1grid=griddedInterpolant(I1,'linear','none');
[l,w,h]=size(I1); 
I2 = zeros(l,w,h);
R  = T(1:3,1:3);
R_ = inv(R);
t  = T(1:3,4);

[x2,y2,z2] = ind2sub([l,w,h],(1:l*w*h)');
I2_xyz     = [x2,y2,z2];
P1         = (I2_xyz-kron(ones(l*w*h,1),t'))*R_';

index1 = find(P1(:,1)>=1 & P1(:,1)<=l &...
              P1(:,2)>=1 & P1(:,2)<=w &...
              P1(:,3)>=1 & P1(:,3)<=h );
P1     = P1(index1,:);
index2 = find(I1grid(P1));
P1     = P1(index2,:);
P2     = I2_xyz(index1(index2),:);
I2(index1(index2)) = I1grid(P1);
end
