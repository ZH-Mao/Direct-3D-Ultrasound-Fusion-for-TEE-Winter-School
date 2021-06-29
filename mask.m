function I=mask(Image, m)
% The code is used to generate a mask matrix which main area is white(255)
% Image: input image
% m: exclude m-pixel-width on the image boundary
% Author: Zhehua Mao {zhehua.mao@student.uts.edu.au}
% Created on 20201222
% Version: 1.0
%==========================================================================
[L,W,H]=size(Image);
Image(1,:,:)=0;
Image(L,:,:)=0;
Image(:,1,:)=0;
Image(:,W,:)=0;
Image(:,:,1)=0;
Image(:,:,H)=0;
res=Image;
if m==0
    res(Image~=0)=255;
    I=double(res);
else
    for n=1:m
        for k=2:H-1
            for j=2:W-1
                for i=2:L-1
                    if Image(i,j,k)==0
                        continue;
                    end
                    if (Image(i-1,j,k)==0)||(Image(i+1,j,k)==0)||(Image(i,j-1,k)==0)||(Image(i,j+1,k)==0)||(Image(i,j,k-1)==0)||(Image(i,j,k+1)==0)
                        res(i,j,k)=0;
                    end
                end
            end
        end
        Image=res;
    end
    I=double(res);
    I(I~=0)=255;
end
end