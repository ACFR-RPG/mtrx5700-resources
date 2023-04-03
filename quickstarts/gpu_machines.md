# GPU Enabled Lab Machines
In this course we have access to two remote lab machines with GPUs to train and run machine learning code. These are available to students and may be logged into using your standard lab machine username and password.

## IP Addresses

The two machines are abled to be accessed using:

```bash
ssh <USER_NAME>@kirbylab-gpu-0X.mxlab.acfr.usyd.edu.au
```
Where X is one of 1 or 2 indicating the machine number. You will need to be on the University of Sydney VPN to access these resources.

## Managing Runtime
To be as fair as possible, these machines will run with different processes on each GPU. That means each machine may have at most 2 groups running things on each machine at each time. There will be a Google sheet available from the MTRX5700 staff with the machine and its GPU(s) which you are to check and schedule into. Where groups are seen to hog the GPU, their processes will be killed automatically to ensure fair and equal availability to the class.  

For major projects, we expect that not all of the class will seek to use a machine learning component, and the runtimes for this will be managed accordingly.

## Machine Details
| Spec     | kirbylab-gpu-01 | kirbylab-gpu-02 |
| ----------- | ----------- | ----------- |
| CPU      | i7-7820X (16) @ 4.6GHz |  i7-5930K (12) @ 3.7GHz     |
| GPU 0   | RTX3080        | GTX1080 |
| GPU 1   | RTX3080        | GTX1080|
| RAM   | 32GB        | 64GB
| System   | Ubuntu 20.04.5        |Ubuntu 20.04.5 |


`gpu-02` is likely best for things like SLAM with feature detection running on the GPU, since it has higher memory, while `gpu-01` might be better for depth estimation or pretraining an object detection/segmentation network.

## Moving Datasets
Your user directory is synced between the Lab PCs and the GPU enabled machines. As such, you can place your datasets in your home directory, and your code and it will appear shortly in the directory of the GPU machines. Note it has to be in the root of your home directory - do not place in Downloads!


## Running Restricted to Single GPU

The following bash script might be useful to you in running training code restricted to a single GPU. You should do this where possible. Do not hog GPUs.

```bash
#!/bin/bash

export CUDA_VISIBLE_DEVICES=X   #Where X is 0 or 1

python3 train.py 
```

You should then call `./train.sh` to execute the above script from the same folder as your python script (assuming you've called it this). This ensures pytorch only ever ses the GPU which has been set to visible.

Note you may need to chmod +x the shell script.
