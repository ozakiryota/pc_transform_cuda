# pc_transform_cuda
## Demo
x + 1 m, y + 2 m, z + 3 m, roll + 10 deg, pitch + 20 deg, yaw + 30 deg
![pc_transform_cuda](https://user-images.githubusercontent.com/37431972/163698277-8404b96e-cf79-46bc-b0c1-4872f06b7e50.png)

## Installation
### Build locally
Requirements:
* ROS
* PCL
* nvidia-cuda-toolkit

```bash
cd ~/catkin_ws/src
git clone https://github.com/ozakiryota/pc_transform_cuda.git
cd ..
catkin_make
```

### Build with Docker
Requirements:
* Docker

```bash
git clone https://github.com/ozakiryota/pc_transform_cuda.git
cd pc_transform_cuda/docker
./build.sh
```

## Usage
1. Edit launch file
2. Run
```bash
roslaunch pc_transform_cuda cuda_hdl32e.launch
```
