# transformable_quadruped_wheelchair_unity


## Overview
Current wheelchairs, relying on wheels for movement, struggle with uneven terrains and steps. This project focuses on the integration of a four-legged robot and a wheelchair to create the Transformable Quadruped Wheelchair, designed to ascend and descend stairs easily even with a rider on board. This is achieved using reinforcement learning, marking a rare experiment that, if successful, could significantly enhance mobility freedom for many. As an initial step, this study experimented with mounting a Tesla Bot 3D model in place of a human on the Transformable Quadruped Wheelchair, to acquire the movements of ascending and descending straight stairs. The code used in these experiments is shared in this repository.

## Rider Information
The 3D model of the Tesla Bot used as a rider can be installed from this page: Tesla Bot 3D Model. Currently, it seems that it can no longer be installed from this site but is available for purchase on other websites. When experimenting, it is necessary to install it and adjust the joint angles properly to position it in a sitting posture.

## How to Use
When using the system, switch between learning the action of climbing up or descending stairs, then proceed with the training using the PPO algorithm from ML-Agents.

``` transformable-quadruped-wheelchair.yaml
behaviors:
  TransformableQuadrupedWheelchair:
    trainer_type: ppo
    hyperparameters:
      batch_size: 1024
      buffer_size: 100000
      learning_rate: 0.0003
      beta: 0.001
      epsilon: 0.2
      lambd: 0.99
      num_epoch: 3
      learning_rate_schedule: linear
    network_settings:
      normalize: true
      hidden_units: 2048
      num_layers: 5
      vis_encode_type: simple
    reward_signals:
      extrinsic:
        gamma: 0.99
        strength: 1.0
    keep_checkpoints: 5
    max_steps: 100000000
    time_horizon: 2000
    summary_freq: 12000
```

### Ascending Action
For ascending, in the QuadrupedAgents script attached to the Scripts object, set Joy Mode to Autonomous, Wheel Chair Mode to Up Stair, and EnvType to Linear Stair Climbing With Passenger.

![image](https://github.com/AkamisakaAtsuki/transformable_quadruped_wheelchair_unity/blob/main/Assets/Images/stair-ascend.png)


### Descending Action
For descending, in the QuadrupedAgents script attached to the Scripts object, set Joy Mode to Autonomous, Wheel Chair Mode to Down Stair, and EnvType to Linear Stair Descending With Passenger.

![image](https://github.com/AkamisakaAtsuki/transformable_quadruped_wheelchair_unity/blob/main/Assets/Images/stair-descend.png)

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
