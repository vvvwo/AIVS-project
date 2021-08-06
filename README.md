# Approximate Intrinsic Voxel Structure for Point Cloud Simplification

![image](https://user-images.githubusercontent.com/65271555/128493952-cc959564-83af-4a49-8619-58a5466e7389.png)

Personal Website: https://aliexken.github.io/

## Abstract

A point cloud as an information-intensive 3D representation usually requires a large amount of transmission, storage and computing resources, which seriously hinder its usage in many emerging fields. In this paper, we propose a novel point cloud simplification method, Approximate Intrinsic Voxel Structure (AIVS), to meet the diverse demands in real-world application scenarios. The method includes point cloud pre-processing (denoising and down-sampling), AIVS-based realization for isotropic simplification and flexible simplification with intrinsic control of point distance. To demonstrate the effectiveness of the proposed AIVS-based method, we conducted extensive experiments by comparing it with several relevant point cloud simplification methods on three public datasets, including Stanford, SHREC, and RGB-D scene models. The experimental results indicate that AIVS has great advantages over peers in terms of moving least squares (MLS) surface approximation quality, curvature-sensitive sampling, sharp-feature keeping and processing speed.

## EXE

The files in "EXE" can be used to simplified an input point cloud directly.

Using the run.cmd to implement the ".exe"

## Update Infor

2021/07/22

Additional Files：

We add a new pure project "AIVS_Pure" and related "EXE" files into the repository.

2020/04/28

Second Version:

The point cloud simplification based on AIVS (speed up by OpenMP)

2020/04/07

First Version:

The point cloud simplification based on AIVS (a stable version without parallel computation structure)

