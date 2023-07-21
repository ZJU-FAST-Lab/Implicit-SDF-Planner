# Implicit-SDF-Planner

## Continuous Implicit SDF Based Any-shape Robot Trajectory Optimization

Our paper is accepted at IROS2023, and we will release some of our code around the end of July.

Preprint: [arxiv](https://arxiv.org/abs/2303.01330)

* [Video on Youtube](https://www.youtube.com/watch?v=Sb6HaVMZWak&ab_channel=FeiGao) or [Video on Bilibili](https://www.bilibili.com/video/BV1Rg4y1E79w/?spm_id_from=333.999.0.0)

<a href="https://www.youtube.com/watch?v=Sb6HaVMZWak&ab_channel=FeiGao" target="blank">
  <p align="center">
    <img src="fig/head.png" width="800"/>
  </p>
</a>

## Demos supported by our pipeline

- Some of the demos are our **further work** after the IROS conference, and a more general pipeline for whole-body planning of arbitrarily shaped robots with **Continuous Collision Safety Certification** is on the way.
  
  #### Arbitrarily shaped robots fly over three consecutive narrow slits (with quadrotor dynamics)
  
  <div align="center">
  <img src="fig/3Ddemo1.gif" width="48%" />
  <img src="fig/3Ddemo2.gif" width="48%" />
  </div>
  <br>
  <div align="center">
  <img src="fig/3Ddemo3.gif" width="48%" />
  <img src="fig/3Ddemo4.gif" width="48%" />
  </div>

#### Arbitrarily shaped robots traverse the random map (with rotation decoupled from translation)

<div align="center">
  <img src="fig/2Ddemo1.gif" width="48%" />
  <img src="fig/2Ddemo2.gif" width="48%" />
</div>
<br>
<div align="center">
  <img src="fig/2Ddemo3.gif" width="48%" />
  <img src="fig/2Ddemo4.gif" width="48%" />
</div>

#### Continuous Collision Avoidance with Safety Certification (with rotation decoupled from translation)

<div align="center">
  <img src="fig/CCA1.gif" width="95%" />
</div>
<br>
<div align="center">
  <img src="fig/CCA2.gif" width="95%" />
</div>

#### Obstacle avoidance planning for shape variant robots (with quadrotor dynamics)

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
roslaunch ego_planner demox.launch #(x=1,2,3...)
```