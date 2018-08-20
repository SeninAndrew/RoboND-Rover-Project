# Introduction

The purpose of this project is to write the code to drive the rover in the simulated Unity environment. The goal of the rover is to create maps of the terrain where it is placed to and to collect the yellow rocks. The code consists of 2 parts: a jupyter notebook which is used to debug the perception part of the project and Python scripts which perform the actual controlling of the rover. 

# Jupyter notebook

Please use Rover_Lab_Notebook_v2.ipynb file which is based on the "Online: Rover Lab" page of the course rather than Rover_Project_Test_Notebook.ipynb from the github repository.

## Image wrapping

In order to project the image visible from the camera mounted on the rover to the top-down view we use the OpenCV projective transform function. We can do it by manually setting correspondence between 4 points on the original image to 4 points on the top-down view (here we assume the 4 points make a rectangular and all of them are located on a plane). This code is not modified from the lecture notes. 

[image_0]: ./misc/perspective.jpg

## Color thresholding

In order to detect drivable terrain and rocks we use color thresholding. For the first part I convert the image to the HSV color space (based on my tests it gives slightly better results) and apply manually selected thresholds - see the color_thresh function. For the 

## Coordinates transformation


