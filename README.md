# Enhanced Detection and Reconstruction of Transparent Objects with Infrared Projection-based RGB-D Cameras

Robotic manipulation systems frequently utilize RGB-D cameras based on infrared projection to perceive three-dimensional environments. Unfortunately, this technique often fails on transparent objects such as glasses, bottles and plastic containers. This enhanced method exploits the perceived infrared camera images to detect and reconstruct volumetric shapes of arbitrary transparent objects. Our reconstruction pipeline first segments transparent surfaces based on pattern scattering and absorption, followed by optimization-based multi-view reconstruction of volumetric object models. Outputs from the segmentation stage can also be utilized for single-view transparent object detection. The presented methods improve on previous work by analyzing infrared camera images directly and by successfully reconstructing cavities in objects such as drinking glasses. 
A dataset of recorded transparent objects, autonomously gathered by a robotic camera-in-hand setup, is published together with this work.

This repository contains code for generating volumetric 3D reconstructions of transparent objects. For detecting transparent objects, see: [here](https://github.com/Misterfun88/glass-recognition-enhanced).
The dataset will be published [here](https://tams.informatik.uni-hamburg.de/research/datasets/index.php).

## Citation

```@
{inproceedings{
iccsip2020,
title={Detection and Reconstruction of Transparent Objects with Infrared Projection-based RGB-D Cameras},
author={Misterfun88},
booktitle={International Conference on Cognitive Systems and Information Processing (ICCSIP)},
year={2020}
}
```