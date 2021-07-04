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

%==================INPUT==========================
Framename1      = 'vol1';
Framename2      = 'vol2';

I1 = sprintf('%s.mat',  Framename1);
I2 = sprintf('%s_enlarged.mat',  Framename2);
%=================================================

I1=importdata(I1); %read frame 1
I1=double(I1);
I1_grid=griddedInterpolant(I1,'linear','none'); %Interpolation

I2=importdata(I2);  %read frame 2
I2=double(I2);
I2_grid=griddedInterpolant(I2,'linear','none'); %Interpolation

[a,b,c]=gradient(I2); 
I2_x=griddedInterpolant(b,'linear','none'); %Interpolation of gradient of intensity along x
I2_y=griddedInterpolant(a,'linear','none'); %Interpolation of gradient of intensity along y
I2_z=griddedInterpolant(c,'linear','none'); %Interpolation of gradient of intensity along z
[l_,w_,h_]=size(I2);

T = eye(4); % Initializa the pose (Transformation matrix)

mask1=mask(I1,10);  % mask matrix frame 1
mask2=mask(I2,10);  % mask matrix frame 2
mask2_grid=griddedInterpolant(mask2,'linear','none'); % Interpolate mask2

% obtian coordinates of valid voxels of vol1
index = find(mask1~=0);
[l,w,h]    = size(I1);
[x1,y1,z1] = ind2sub([l,w,h],index);
I1_xyz     = [x1,y1,z1];

d=ones(6,1);
m=1;

Iteration = 300;

Optimization.iteration  = zeros(1,Iteration);
Optimization.objvalues  = zeros(1,Iteration);
Optimization.steplength = zeros(1,Iteration);

tic;
while (norm(d)>1e-10) && (m<=Iteration)
    R = T(1:3,1:3);
    t = T(1:3,4);
    P2 = I1_xyz*R' + kron(ones(size(I1_xyz,1),1),t'); % calculate the corresponding coordinates of I1_xyz in I2 frame.
    
    index1 = find(P2(:,1)>=1 & P2(:,1)<=l_ &...
                  P2(:,2)>=1 & P2(:,2)<=w_ &...
                  P2(:,3)>=1 & P2(:,3)<=h_ );
    P2      = P2(index1,:);
    isWhite = mask2_grid(P2);
    index2  = find(isWhite==255);
    P2      = P2(index2,:);
    P1      = I1_xyz(index1(index2),:);
    num     = size(P1,1);
    e       = I1_grid(P1)- I2_grid(P2); % calculate overall intensity differences
      
    %=======Participants need to complete the following function===========
    
    J  = jacobian_Mat();       % calculate overall Jacobian matrix
    
    %===========================END========================================
    
    e  = sparse(e);
    J  = sparse(J);
    d  =(J'*J)\(-J'*e);  % calculate step change using GN equation
    dT = Lie2T(d);       % convert Lie algebra to Eucludian Matrix
    T  = dT*T;           % update pose
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
fprintf('The transforming matrix from %s to %s is\n\n', Framename1,Framename2);
disp(T);
R=T(1:3,1:3);
t=T(1:3,4);
Li_=R_t2exp(R,t);
theta=(rotm2eul(R, 'XYZ'))';
trans=t;
savefile = sprintf('Pose_%s_to_%s.mat', Framename1,Framename2);
save( savefile,'Li_','T', 'theta', 'trans','Optimization');
