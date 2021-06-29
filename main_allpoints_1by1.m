% ===============================================================================================
% Direct registration and fusion of 3D TEE images
% Version:2.0
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

clear;
close all;
clc;

format long;
% addpath('/data/zhmao/Datasets20201012/LYH_ECG');
addpath('/data/zhmao/1_Static/Initialization_Zhehua');


%==================INPUT==========================
PatientName     = 'LYH';
Framename1      = 'T215F01';
Framename2      = 'T216F01';

I1 = sprintf('%s_%s_enlarged.mat', PatientName, Framename1);
I2 = sprintf('%s_%s_enlarged.mat', PatientName, Framename2);
%=================================================

I1=importdata(I1); %read frame 1
I1=double(I1);
I1_grid=griddedInterpolant(I1,'linear','none');

I2=importdata(I2);
I2=double(I2);
I2_grid=griddedInterpolant(I2,'linear','none');

[I2x,I2y,I2z]=gradient(I2);
I2_x=griddedInterpolant(I2x,'linear','none');
I2_y=griddedInterpolant(I2y,'linear','none');
I2_z=griddedInterpolant(I2z,'linear','none');
[l_,w_,h_]=size(I2);

T = eye(4); % Initialization

mask1=mask(I1,10);
mask2=mask(I2,10);
mask2_grid=griddedInterpolant(mask2,'linear','none');

index = find(mask1~=0);
[l,w,h]    = size(I1);
[x1,y1,z1] = ind2sub([l,w,h],index);
I1_xyz     = [x1,y1,z1];

d=ones(6,1);
m=1;

Iteration = 300;

Optimization.iteration =zeros(1,Iteration);
Optimization.objvalues=zeros(1,Iteration);
Optimization.steplength=zeros(1,Iteration);


tic;
while (norm(d)>1e-10) && (m<=Iteration)
    R = T(1:3,1:3);
    t = T(1:3,4);
    P2 = I1_xyz*R' + kron(ones(size(I1_xyz,1),1),t');
    
    index1 = find(P2(:,1)>=1 & P2(:,1)<=l_ &...
                  P2(:,2)>=1 & P2(:,2)<=w_ &...
                  P2(:,3)>=1 & P2(:,3)<=h_ );
    P2      = P2(index1,:);
    isWhite = mask2_grid(P2);
    index2  = find(isWhite==255);
    P2      = P2(index2,:);
    P1      = I1_xyz(index1(index2),:);
    num     = size(P1,1);
    e       = I1_grid(P1)- I2_grid(P2);

    a1 = P2(:,1); b1=P2(:,2);  c1=P2(:,3);
    a2 = I2_y(P2); b2=I2_x(P2); c2=I2_z(P2);
    J  = -[a2,b2,c2,b1.*c2-c1.*b2,c1.*a2-a1.*c2,a1.*b2-b1.*a2];
    e  = sparse(e);
    J  = sparse(J);
    d =(J'*J)\(-J'*e);
    dT = Lie2T(d);
    T  = dT*T;
    objvalues  = e'*e;
    steplength = norm(d);
    
    disp(m);
    disp('Values of objective function is:');
    disp(objvalues);
    disp('Step length is:');
    disp(steplength);
    
    Optimization.iteration(m) = m;
    Optimization.objvalues(m) = objvalues;
    Optimization.steplength(m)= steplength;
    
    m=m+1;
end
Optimization.timecost=toc;
fprintf('The transforming matrix from %s_%s to %s_%s is\n\n',PatientName, Framename1,PatientName,Framename2);
disp(T);
R=T(1:3,1:3);
t=T(1:3,4);
Li_=R_t2exp(R,t);
theta=(rotm2eul(R, 'XYZ'))';
trans=t;
savefile = sprintf('%s_Pose_allpts_%s_to_%s.mat',PatientName, Framename1,Framename2);
save( savefile,'Li_','T', 'theta', 'trans','Optimization');