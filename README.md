# lidar-road-segmentation
Dumb experiments for road segmentation using Point Cloud from LiDAR

No high-math, neural networks or ready solution. Only dumb approaches like 
 - mark everything with specific height as road
 - mark every occupacity grid cell with low height dispersion as a road
 - start breadth-first search from the center (LiDAR location) and mark neighbour cells as road if there is a small height change etc
 - Maybe, RANSAC
 
 I'm not sure, but current (master) implementation is a dumb RANSAC
