# DeepPilot: A CNN for Autonomous Drone Racing

Autonomous Drone Racing (ADR) was first proposed in IROS 2016. It called for the development of an autonomous drone capable of beating a human in a drone race. After almost five years, several teams have proposed different solutions with a common pipeline: gate detection; drone localization; and stable flight control. Recently, Deep Learning (DL) has been used for gate detection and localization of the drone regarding the gate. However, recent competitions such as the Game of Drones, held at NeurIPS 2019, called for solutions where DL played a more significant role. Motivated by the latter, in this work, we propose a CNN approach called DeepPilot that takes camera images as input and predicts flight commands as output. These flight commands represent: the angular position of the drone’s body frame in the roll and pitch angles, thus producing translation motion in those angles; rotational speed in the yaw angle; and vertical speed referred as altitude h. Values for these 4 flight commands, predicted by DeepPilot, are passed to the drone’s inner controller, thus enabling the drone to navigate autonomously through the gates in the racetrack. For this, we assume that the next gate becomes visible immediately after the current gate has been crossed. We present evaluations in simulated racetrack environments where DeepPilot is run several times successfully to prove repeatability. In average, DeepPilot runs at 25 frames per second (fps). We also present a thorough evaluation of what we called a temporal approach, which consists of creating a mosaic image, with consecutive camera frames, that is passed as input to the DeepPilot. We argue that this helps to learn the drone’s motion trend regarding the gate, thus acting as a local memory that leverages the prediction of the flight commands. Our results indicate that this purely DL-based artificial pilot is feasible to be used for the ADR challenge.

# Overview of our approach

![alt text](https://github.com/QuetzalCpp/DeepPilot/blob/master/images/overview_approach.jpg)

It consists of 4 steps: (1) Data acquisition using the drone’s onboard camera; (2) Real-time mosaic generation, consisting of 6 frames; (3) Flight commands prediction using our proposed CNN named DeepPilot, these commands are represented by the tuple (ϕ,θ,ψ,h); (4) Implementation of a filter to smooth the signal. A video illustrating the performance of our proposed DeepPilot can be found at https://youtu.be/Qo48pRCxM40.

[![Watch the video](https://i9.ytimg.com/vi/YD5oqe8DelE/mq1.jpg?sqp=COCt0_kF&rs=AOn4CLDaF68MYig0YzuYo4j69i4LAhMH_w)](https://www.youtube.com/watch?v=YD5oqe8DelE)
[![Watch the video](https://i9.ytimg.com/vi/YD5oqe8DelE/mq2.jpg?sqp=COCt0_kF&rs=AOn4CLC7MXWHPWOAqlvyi7S3XoJCjyNkCw)](https://www.youtube.com/watch?v=YD5oqe8DelE)
[![Watch the video](https://i9.ytimg.com/vi/YD5oqe8DelE/mq3.jpg?sqp=COCt0_kF&rs=AOn4CLALymXsHMEkw7ccui0UE9yMRHh7ag)](https://www.youtube.com/watch?v=YD5oqe8DelE)


# DeepPilot Architecture
Our proposed DeepPilot runs 3 specialized models in parallel. The first one predicts ϕ and θ angular positions of the body frame; the second one predicts ψ, the rotational speed over the Z-axis; and the third one predicts h, the vertical speed. The size of the kernels is indicated in the colored boxes at the bottom-left.

![alt text](https://github.com/QuetzalCpp/DeepPilot/blob/master/images/DeepPilot_architecture.jpg)




