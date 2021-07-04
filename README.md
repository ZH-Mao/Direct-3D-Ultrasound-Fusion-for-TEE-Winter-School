# Direct-3D-Ultrasound-Fusion-for-TEE-Winter-School
This repository include codes for IEEE-UTS Winter School-2021

Project step: The participants will be asked to complete the following exploration steps based on the provided materials (ECG-gated 3D TEE data*, designated software, codes, etc.)

1. Image registration (intensity-based methods):
In this step, participants will implement an intensity-based image registration algorithm or use designated software-3D Slicer to estimate transformation matrices (relative poses) between image frames. A part of codes are available in this repository. Please complete the main script-Main_intensityBased.m.

2. Image Fusion:
In this step, participants can choose any methods to composite the aligned images. An easiest method is to average the intensity of images. The image averaging code ImageAverage.m is available in this repository.

3. Panoramic image reconstruction:
In this step, participants will learn to use a sequential strategy to register and fuse a sequence of 3D TEE images. In the end, a panoramic image with a larger FoV than the original single volume will be reconstructed.

4. Evaluation (optional):
In this step, participants can use some estimation metrics such as Hausdorff distance (HD) which are available in MeshLab to evaluate the accuracy of the registration. You can download MeshLab from https://www.meshlab.net/#download. And for HD, you can refer the following webpage.
http://meshlabstuff.blogspot.com/2010/01/measuring-difference-between-two-meshes.html

Main reference: Mao, Z., Zhao, L., Huang, S., Fan, Y., & Lee, A. P. W. (2021). Direct 3D Ultrasound Fusion for Transesophageal Echocardiography. Computers in Biology and Medicine, 104502. https://doi.org/10.1016/j.compbiomed.2021.104502

*All participants must keep the clinical data confidential, you cannot publish, copy, sell, rent, transfer, promulgate, let out, disclose, or reveal the confidential datasets.
