# Project: Calib-Mkinit
An algorithm for odometric and extrinsic calibration.

# Inputs
1. Odometric measurements: encoder c_l c_r
2. Mark observation measurements: SE(3)

# Outputs
1. Extrinsic parameters: Tbc, SE(3)
2. Odometric parameters: [C], 2x2 matrix

# Algorithm

## 1. Estimate ground plane w.r.t. camera
As in my previous paper on IEEE Sensors Journal.

## 2. Project to ground plane
Project everything to the ground plane SE(2), including extrinsic parameters, mark observation, and mark locations if necessary.

1. Define a projected camera frame {d}, compute Tdc. Now, in extrinsic parametrers, Tbd belongs to SE(2). 
2. Refine mark observation measurements, compute ground plane w.r.t each mark, and refine the measurements according to the constraints
3. Project mark observation measurements from SE(3) to SE(2)

## 3. Estimate rotational terms in [C]
Get robot rotation from mark observation, and compute 2 terms in C

## 4. Estimate remaining terms: 
1 scale term in [C], 3 dof Tbd. Use SVD to find the eigen vector with the smallest eigen value. Consider cos(yaw) and sin(yaw) as 2 variables. Determine the final results by constraint between cos and sin.

# Todo

## Develop comparison algorithm

As in reference paper *A non-iterative and effective procedure for simultaneous odometry and camera calibration for a differential drive mobile robot based on the singular value decomposition* by *Gianluca Antonelli*.

## Develop auto init and joint opt for AGV experiment

The odometric model is different. Some changes should be made in both auto init and joint opt. In joint opt, k_lin and k_rot could be considered as variables. In auto init, still some works to do.