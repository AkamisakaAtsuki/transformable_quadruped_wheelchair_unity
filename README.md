# transformable_quadruped_wheelchair_unity


## Overview
Current wheelchairs, relying on wheels for movement, struggle with uneven terrains and steps. This project focuses on the integration of a four-legged robot and a wheelchair to create the Transformable Quadruped Wheelchair, designed to ascend and descend stairs easily even with a rider on board. This is achieved using reinforcement learning, marking a rare experiment that, if successful, could significantly enhance mobility freedom for many. As an initial step, this study experimented with mounting a Tesla Bot 3D model in place of a human on the Transformable Quadruped Wheelchair, to acquire the movements of ascending and descending straight stairs. The code used in these experiments is shared in this repository.

## Rider Information
The 3D model of the Tesla Bot used as a rider can be installed from this page: Tesla Bot 3D Model. Currently, it seems that it can no longer be installed from this site but is available for purchase on other websites. When experimenting, it is necessary to install it and adjust the joint angles properly to position it in a sitting posture.

## How to Use
When using the system, switch between learning the action of climbing up or descending stairs, then proceed with the training using the PPO algorithm from ML-Agents.

### Ascending Action
For ascending, in the QuadrupedAgents script attached to the Scripts object, set ○○ to △ and ◇ to □.
### Descending Action
For descending, in the QuadrupedAgents script attached to the Scripts object, set ○○ to △ and ◇ to □.
### Executing Training
For training, save the config as ○○ and execute the training.
## Experiment Details
Results are presented as follows:

### Ascending Experiment: Procedure, Data Used, Evaluation Criteria.
### Descending Experiment: Procedure, Data Used, Evaluation Criteria.

## Acknowledgements
Most of the programming for this study was done by me, however, for Lidar.cs, I utilized the program once published by VTC on Unity, which can be found here: VTC on Unity. I express my gratitude.

## License
Apache License
