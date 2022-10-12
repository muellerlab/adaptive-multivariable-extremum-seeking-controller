## Model-Free Online Motion Adaptation for Energy-Efficient Flight of Multicopters with Adaptive Multivariable Extreme Seeking Controller

### Introduction
This repository contains the example code for adaptive multivariable extreme seeking controller that we exploit to seek for optimal speed or sideslip to maximize either endurance or range.

### Getting Started
Both python2 and python3 are allowed to run the example code without no additional dependency other than `numpy` and `matplotlib`.

### Test
Run `python adaptive_multivariable_extreme_seeking_controller/extreme_seeking_control_test.py`.

### Important Notes
The `cost_function()` is assumed to be known in this simulation code. In fact, this function is assumed to be unknown based on model-free setup in this paper. Readers shall also be aware that this function is never explicit used in the simulation code.

### References
If you find this project useful in your work, please consider citing following paper:
"Model-Free Online Motion Adaptation for Energy-Efficient Flight of Multicopters." [[IEEE]](https://ieeexplore.ieee.org/document/9795278)[[arXiv]](https://arxiv.org/abs/2108.03807)
```
@article{wu2022modelfree,
    author={Wu, Xiangyu and Zeng, Jun and Tagliabue, Andrea and Mueller, Mark W.},
    journal={IEEE Access}, 
    title={Model-Free Online Motion Adaptation for Energy-Efficient Flight of Multicopters}, 
    year={2022},
    volume={10},
    number={},
    pages={65507-65519},
}
```