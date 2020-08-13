# DeepPilot: A CNN for Autonomous Drone Racing

Autonomous Drone Racing (ADR) was first proposed in IROS 2016. It called for the development of an autonomous drone capable of beating a human in a drone race. After almost five years, several teams have proposed different solutions with a common pipeline: gate detection; drone localization; and stable flight control. Recently, Deep Learning (DL) has been used for gate detection and localization of the drone regarding the gate. However, recent competitions such as the Game of Drones, held at NeurIPS 2019, called for solutions where DL played a more significant role. Motivated by the latter, in this work, we propose a CNN approach called DeepPilot that takes camera images as input and predicts flight commands as output. These flight commands represent: the angular position of the drone’s body frame in the roll and pitch angles, thus producing translation motion in those angles; rotational speed in the yaw angle; and vertical speed referred as altitude h. Values for these 4 flight commands, predicted by DeepPilot, are passed to the drone’s inner controller, thus enabling the drone to navigate autonomously through the gates in the racetrack. For this, we assume that the next gate becomes visible immediately after the current gate has been crossed. We present evaluations in simulated racetrack environments where DeepPilot is run several times successfully to prove repeatability. In average, DeepPilot runs at 25 frames per second (fps). We also present a thorough evaluation of what we called a temporal approach, which consists of creating a mosaic image, with consecutive camera frames, that is passed as input to the DeepPilot. We argue that this helps to learn the drone’s motion trend regarding the gate, thus acting as a local memory that leverages the prediction of the flight commands. Our results indicate that this purely DL-based artificial pilot is feasible to be used for the ADR challenge.

<p align="center">
  <img src="images/gate5.gif">
</p>

## Overview of our approach

![alt text](https://github.com/QuetzalCpp/DeepPilot/blob/master/images/overview_approach.jpg)

It consists of 4 steps: (1) Data acquisition using the drone’s onboard camera; (2) Real-time mosaic generation, consisting of 6 frames; (3) Flight commands prediction using our proposed CNN named DeepPilot, these commands are represented by the tuple (ϕ,θ,ψ,h); (4) Implementation of a filter to smooth the signal.

## DeepPilot Architecture
Our proposed DeepPilot runs 3 specialized models in parallel. The first one predicts ϕ and θ angular positions of the body frame; the second one predicts ψ, the rotational speed over the Z-axis; and the third one predicts h, the vertical speed. The size of the kernels is indicated in the colored boxes at the bottom-left.

![alt text](https://github.com/QuetzalCpp/DeepPilot/blob/master/images/DeepPilot_architecture.jpg)

## Video
A video of this approach can be watched at: https://youtu.be/Qo48pRCxM40.

[![Watch the video](https://i9.ytimg.com/vi/YD5oqe8DelE/mq2.jpg?sqp=COCt0_kF&rs=AOn4CLC7MXWHPWOAqlvyi7S3XoJCjyNkCw)](https://www.youtube.com/watch?v=YD5oqe8DelE)
[![Watch the video](images/gate5.gif)](https://www.youtube.com/watch?v=YD5oqe8DelE)
[![Watch the video](https://i9.ytimg.com/vi/YD5oqe8DelE/mq3.jpg?sqp=COCt0_kF&rs=AOn4CLALymXsHMEkw7ccui0UE9yMRHh7ag)](https://www.youtube.com/watch?v=YD5oqe8DelE)

## Recommended system
- Ubuntu 16.04
- ROS kinetic Kame
- Python 2.7.15
- Cuda 9.0
- Cudnn 7.3.0
- Tensorflow 1.12.0
- Keras 2.2.4
- Tum_simulator ported to Kinetic (https://github.com/angelsantamaria/tum_simulator.git)

## Datasets to train DeepPilot
To be announced soon

## Reference
If you use any of data, model or code, please cite the following reference:

Rojas-Perez, L.O., & Martinez-Carranza, J. (2019). DeepPilot: A CNN for Autonomous Drone Racing. Sensors, 20(16), 4524.
https://doi.org/10.3390/s20164524

```
@article{oyuki2020deeppilot,
  title={DeepPilot: A CNN for Autonomous Drone Racing.},
  author={Rojas-Perez, L.Oyuki and Martinez-Carranza, Jose},
  journal={Sensors},
  volume={20},
  number={16},
  pages={4524},
  year={2020}
}
```
 ## Acknowledgements
We are thankful for the processing time granted by the National Laboratory of Supercomputing (LNS) under the project 201902063C. The first author is thankful to Consejo Nacional de Ciencia y Tecnología (CONACYT) for the scholarship No. 924254. We are also thankful for the partial financial support granted via the FORDECYT project no. 296737 “Consorcio en Inteligencia Artificial” for the development of this work.


