# ros_morphology_robot
This repository is the code accompanying the manuscript "Low-cost, autonomous microscopy using deep learning and robotics: A crystal morphology case study" published in Engineering Application of Artificial Intelligence (EAAI) (<https://doi.org/10.1016/j.engappai.2023.106985>). It contains the source code for running the robotics platform showcased in the manuscript and providing real time morphology labelling. Please note this code is no longer under active development and so can only be used as found at this repository. 

# Versioning
Please note, the final version is found in the contents of the `v2_robot` directory. The `robot` directory contains legacy code that is here for completeness and the authors reference but it is not encouraged to be used. 

# Usage
To run the code, users must have `roscore` running as per the ROS documentaion (<http://wiki.ros.org/roscore>).
```
roscore
```

In additon to this, users must also run the subscriber node and the controll app simultaneously. 
The control app is found at the following location `v2_robot/src/robot_v2/scripts/control_app.py` and should be run from the scripts directory. 
```
python3 control_app.py
```
The subscriber node should be run from an additional terminal window using the bash file included at the following location `v2_robot/sub.sh`.
```
bash sub.sh
```

# Bring-Your-Own-Model
Users can suppy their own custom trained neural networks by exporting their trained model as `.pkl` files. These should then replace the saved model included in this repositry which can be found at `v2_robot/src/robot_v2/scripts/src/model.pkl`

# Model training
As mentioned in the manuscript, the code for training the morphology classification network can be found in a different reposity as this was developed offline before the model was carried over of the live tests. The offline training can be found at (<https://github.com/MRW-Code/ai_robotics_morphology_prediction>).
