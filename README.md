# Implicit-SDF-Planner

**Related Paper**: 

- [Continuous Implicit SDF Based Any-shape Robot Trajectory Optimization](https://arxiv.org/abs/2303.01330), Tingrui Zhang*, Jingping Wang*,Chao Xu, Alan Gao, Fei Gao.
- Our paper is accepted at IROS2023, and we will release some of our code around the end of July.

If you find this work useful in your research, please consider citing:

```
@INPROCEEDINGS{10342104,
  author={Zhang, Tingrui and Wang, Jingping and Xu, Chao and Gao, Alan and Gao, Fei},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Continuous Implicit SDF Based Any-Shape Robot Trajectory Optimization}, 
  year={2023},
  volume={},
  number={},
  pages={282-289},
  doi={10.1109/IROS55552.2023.10342104}}
```

* [Video on Youtube](https://www.youtube.com/watch?v=Sb6HaVMZWak&ab_channel=FeiGao) or [Video on Bilibili](https://www.bilibili.com/video/BV1Rg4y1E79w/?spm_id_from=333.999.0.0)

<a href="https://www.youtube.com/watch?v=Sb6HaVMZWak&ab_channel=FeiGao" target="blank">
  <p align="center">
    <img src="fig/head.png" width="800"/>
  </p>
</a>

## Demos supported by our pipeline

- Some of the demos are our **further work** after the IROS conference, and a more general pipeline for whole-body planning of arbitrarily shaped robots with **Continuous Collision Safety Certification** is on the way.
  
  #### 3D - Arbitrarily shaped robots fly over three consecutive narrow slits (with quadrotor dynamics)
  
  <div align="center">
  <img src="fig/3Ddemo1.gif" width="48%" />
  <img src="fig/3Ddemo2.gif" width="48%" />
  </div>
  <br>
  <div align="center">
  <img src="fig/3Ddemo3.gif" width="48%" />
  <img src="fig/3Ddemo4.gif" width="48%" />
  </div>

#### 2D - Arbitrarily shaped robots traverse the random map (with rotation decoupled from translation)

<div align="center">
  <img src="fig/2Ddemo1.gif" width="48%" />
  <img src="fig/2Ddemo2.gif" width="48%" />
</div>
<br>
<div align="center">
  <img src="fig/2Ddemo3.gif" width="48%" />
  <img src="fig/2Ddemo4.gif" width="48%" />
</div>

#### 2D - Continuous Collision Avoidance with Safety Certification (with rotation decoupled from translation)



<div align="center">
  <img src="fig/2Ddemo.gif" width="95%" />
</div>
<br>
<div align="center">
  <img src="fig/CCA1.gif" width="95%" />
</div>
<br>
<div align="center">
  <img src="fig/CCA2.gif" width="95%" />
</div>

#### 3D - Obstacle avoidance planning for shape variant robots (with quadrotor dynamics)

<div align="center">
  <img src="fig/scaledemo1.gif" width="95%" />
</div>
<br>
<div align="center">
  <img src="fig/scaledemo2.gif" width="95%" />
</div>

Compiling tests passed on ubuntu 20.04 with ros installed. You can just execute the following commands one by one.

## Qucik Start

```sh
echo "alias python=python3" >> ~/.zshrc  #(If use zsh) 
echo "alias python=python3" >> ~/.bashrc #(If use bash)
python3 -m pip3 install pygame==2.0.0.dev12 
sudo apt-get install gcc g++ make gfortran cmake libomp-dev
git clone https://github.com/ZJU-FAST-Lab/Implicit-SDF-Planner.git
cd Implicit-SDF-Planner
./build.sh
source devel/setup.bash #(If use bash)
source devel/setup.zsh  #(If use zsh) 
roslaunch plan_manager demox.launch #(x=1,2,3...)
```

- Then use "3D Nav Goal" in rviz or click on the "G" button on the keyboard to publish the goal for  navigation.
  **Note that the start and end points of the clicks must be within the map.**
  
  ## More Examples (Some-shaped robots with uav dynamics)

### 2D version and shape variant demos will be released later...

### 3D version demos

**You can click buttons on the screen to visualize the swept volume, stop the optimization process early, etc.**

https://github.com/ZJU-FAST-Lab/Implicit-SDF-Planner/assets/83890569/df19362b-2827-4d36-93a2-d9ce9261f44d

Have a cool example? Submit a PR! You can either extend the robot's shape arbitrarily via an obj file in [shapes](src/plan_manager/shapes).The code will automatically use libigl to get its SDF. You can also inherit from Generalshape in [Shape.hpp](src/utils/include/utils/Shape.hpp) to implement the desired shape and the associated SDF methods.For visualization purposes, you will also need a corresponding obj file for the robot shape. 

**Supported Shapes now**

<div style="display: flex; justify-content: center;">
  <table>
    <tr>
      <td><img src="fig/1.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/2.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/3.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/4.png" style="max-width: 120px; padding: 0px;"></td>
    </tr>
    <tr>
      <td><img src="fig/5.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/6.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/7.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/8.png" style="max-width: 120px; padding: 0px;"></td>
    </tr>
    <tr>
      <td><img src="fig/9.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/10.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/11.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/12.png" style="max-width: 120px; padding: 0px;"></td>
    </tr>
    <tr>
      <td><img src="fig/13.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/14.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/15.png" style="max-width: 120px; padding: 0px;"></td>
      <td><img src="fig/16.png" style="max-width: 120px; padding: 0px;"></td>
    </tr>
  </table>
</div>

## Tips

1. We use OpenMp for parallel acceleration, so the "threads_num" in the yaml should be adjusted to improve performance. The recommended threads_num is about 1.5 times the number of logical cores on the machine.
2. If users customize the shape, the obj file must be provided. We recommend using meshlab to generate obj files. For better performance, users can also implement corresponding SDF function, otherwise Libigl is used by default to compute the SDF.
3. The kernel_size multiplied by the map resolution should not be too small, this value should be greater than the maximum length of the robot's shape. So the "kernel_size" in the yaml should be adjusted accordingly (not too small).

## Acknowledgements

**There are several important works which support this project:**

- [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER): An efficient and versatile multicopter trajectory optimizer built upon a novel sparse trajectory representation named [MINCO](https://arxiv.org/pdf/2103.00190v2.pdf).
- [Swept-Volume](https://github.com/sgsellan/swept-volumes): For visualization purpose in our project.
- [LMBM](https://github.com/ZJU-FAST-Lab/LMBM):  Limited Memory Bundle Method for nonsmooth-optimization in our project.

## Licence

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

## Maintaince

For any technical issue or bug, please contact Tingrui Zhang (tingruizhang@zju.edu.cn) or Jingping Wang (22232111@zju.edu.cn).
For commercial inquiries, please contact [Fei GAO](http://zju-fast.com/fei-gao/) (fgaoaa@zju.edu.cn).
