# superpixel_image_stitching
In this paper, the image is spliced based on super pixel.
## Introduction
Super - pixel - based fast image stitching code implementation. We take the superpixel as the alignment unit and separately calculate the mapping transformation matrix of each superpixel. With the same alignment effect as APAP, we can greatly reduce the time of image stitching.<br>
## Requirements
Our program runs on windows8.1. Compile and run with Visual Studio 2015. You need three external dependencies to help you compile and run the code. The three external dependency libraries are:opencv3.3.0、Eigen3 and TinyVLib.<br>
## stitching process
We illustrate the experimental process with the scene of railway tracks. The image stitching process is divided into three steps.
### step1
The image to be spliced is divided into super pixels. The following figure is the result of super-pixel division of the image.<br>
![](https://github.com/GenaWang/superpixel_image_stitching/raw/master/image_show/railtrackMBS_out.jpg)
<br>
### step2
We calculate the mapping matrix for each superpixel.<br>
### step3
We map and transform each super pixel to get a stitching image. The picture below shows the result of stitching.<br>
![](https://github.com/GenaWang/superpixel_image_stitching/raw/master/image_show/railtrack_ours.jpg)
<br>
Here we take a look at the difference between our experiment and the AutoStitching. Here's the AutoStitching result.<br>
![](https://github.com/GenaWang/superpixel_image_stitching/raw/master/image_show/railtrack_AutoStitchMarked.jpg)
<br>
The results of the two images were compared. We can see that our method has the same effect as APAP. Our method can eliminate the problem of ghosting and dislocation in the process of image stitching.
