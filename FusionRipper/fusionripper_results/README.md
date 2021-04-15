# FusionRipper Attack and Analysis Results

The files contain the FusionRipper attack and attack anlaysis results. The attack traces are obtained from the original paper. I thank the author(s) for providing the attack traces. 

These attacks use the ***best attack parameters*** on the real traces from Baidu Apollo and KAIST Complex Urban datasets. 

We analize the spoofing attacks and report the analysis results from the attack traces. 


## Folder Hierarchy

The results are organized in the following hierarchy:

```
|-- attack
|   |-- ba-local
|   |   |-- attack-rhs_691_692_0.6_1.5.csv
|   |   |-- ...
|   |   |-- result.csv
|   |-- ...
|
|-- benign
|   |-- ba-local
|   |   |-- gnss_input.csv
|   |   |-- ground_truth.csv
|   |   |-- imu_input.csv
|   |   `-- lidar_input.csv
|   |-- ...
main.m
postprocess.m
```

- `attack-rhs_691_692_0.6_1.5.csv`
  - `rhs`: attack direction; `691`: starting time for "vulnerability profiling" stage; `692`: starting time for "aggressive spoofing" stage; `0.6`: best attack parameter *d*; `1.5`: best attack parameter *f*.
  - Each row in this file corresponds to a maximum lateral deviation (in meters) in that particular second
- `ground_truth.csv`: benign localization outputs. Below are the description of its content:
- `gnss_input.csv`, `imu_input.csv`, `lidar_input.csv`: Original GPS/IMU/LiDAR input data for MSF.

- `result.csv`
  - this file includes forensic analysis results. 

- `main.m`, `postprocess.m`: matlab scripts to analyze the attack traces and make analysis results. 