
# SDCND : Sensor Fusion and Tracking

## Compute Lidar Point-Cloud from Range Image
### Visualizing range image channels
![Range Image](range_image_channels.png)

### Visualizing point-cloud

|             Front            |             Rear            |
|:----------------------------:|:---------------------------:|
| ![](Front.png "point-cloud") | ![](Rear.png "point-cloud") |

## Birds-Eye View from Lidar PCL

### Converting sensor coordinates to bev-map coordinates
![](BEV-map.png)
### intensity & height layer of bev-map
|             Intensity Map              |            Height Map            |
|:--------------------------------------:|:--------------------------------:|
| ![](img_intensity.png "Intensity Map") | ![](height_map.png "Height Map") |

## Model-based Object Detection in BEV Image
![](labels-detected-objects-screenshot.png)

## Performance Evaluation for Object Detection

![](performance_metrics.png)
![](precision-recall.png)

# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
## Step 1: Implementing an Extended Kalman Filter
EKF has 3 main steps prediction, measurement and update.
- First was adding the representation of the state of the system which consist of process model and velocity.
- working on the prediction function the following matrices were needed (System matrix **F**, process noise covariance **Q**) both these matrices where implemented using the saved parameters in params.py.
- for the update function measurment matrix (H), Gamma residual , covariance of the residual (S), kalman gain(K), state update and covariance update all were calculated to update the state x and covariance P.

## Step 2: Implementing the track management to initialize and delete tracks, set a track state and a track score.

## Step 3: Implementing a single nearest neighbor data association to associate measurements to tracks.

## Step 4: Implementing the nonlinear camera measurement model.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
In Theory:
- **Accuracy**: Cameras and lidar sensors have different strengths and weaknesses. Cameras are better at detecting objects and edges, while lidar sensors are better at measuring depth. By fusing the data from both sensors, it is possible to create a more accurate 3D model of the environment.
- **Reliability**: Cameras and lidar sensors are both susceptible to noise and errors. By fusing the data from both sensors, it is possible to reduce the impact of these errors.
- **Robustness**: Cameras and lidar sensors can be affected by different environmental conditions. For example, cameras can be blinded by bright sunlight, while lidar sensors can be affected by rain or snow. By fusing the data from both sensors, it is possible to create a more robust system that can operate in a variety of conditions.


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
Challenges in real-life scenarios:
- **Data fusion**: Sensor fusion systems must be able to fuse data from multiple sensors in real time. This can be a challenging task, as the data from different sensors may be in different formats, have different resolutions, and be subject to different levels of noise.
- **Environmental conditions**: Sensor fusion systems must be able to operate in a variety of environmental conditions, including rain, snow, fog, and bright sunlight. These conditions can make it difficult for sensors to collect accurate data.
- **Motion**: Sensor fusion systems must be able to operate in a variety of motion conditions, including acceleration, deceleration, and turning. These conditions can cause the sensors to produce inaccurate data.
- **Malfunction**: Sensor fusion systems must be able to handle sensor malfunctions. If a sensor fails, the sensor fusion system must be able to continue operating without the failed sensor.
- **Security**: Sensor fusion systems must be secure from cyberattacks. Attackers could attempt to corrupt the data from the sensors or disrupt the communication between the sensors and the sensor fusion system.

### 4. Can you think of ways to improve your tracking results in the future?

