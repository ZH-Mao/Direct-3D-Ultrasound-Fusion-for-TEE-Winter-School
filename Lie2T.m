function  T=Lie2T(x)
% ===============================================================================================
% Convert x from Lie algbra to transform matrix
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

t_=x(1:3,:);
R_=x(4:6,:);
theta=norm(R_);
n=R_/theta;
n_=[0 -n(3) n(2);n(3) 0 -n(1);-n(2) n(1) 0];
R=cos(theta)*eye(3)+(1-cos(theta))*(n*n')+sin(theta)*n_;
J=(sin(theta)/theta)*eye(3)+(1-sin(theta)/theta)*(n*n')+((1-cos(theta))/theta)*n_;
T_(1,1)={R};
T_(1,2)={J*t_};
T_(2,1)={[0 0 0]};
T_(2,2)={1};
T=cell2mat(T_);
end
