# RoboPhD
Some records of arXiv papers, GRASP seminars, and resources during my PhD period.


## Conference Deadline


- [RSS 2023](https://roboticsconference.org/)

February 3rd, 2023 11:59pm AoE | Paper and Demo Submission Deadline

- [IROS 2023](https://ieee-iros.org/) 

March 1, 2023:  23:59 PST | Deadline for papers submission.

- [CoRL 2023](https://www.corl2023.org/)

May 15, 2023:  Submission and Supplementary materials deadline


## ArXiv Papers

[arXivDaily](https://arxivdaily.com/cate/23/seq/0)

The papers and notes updates weekly, mainly about motion planning

- [2021](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2021.md)

- [2022-s1](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2022-s1.md)

- [2022-s2](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2022-s2.md)

- [2022-s3](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2022-s3.md)

- [2022-s4](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2022-s4.md)

- [2023-s1](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2023-s1.md)

- [2023-s2](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2023-s2.md)

## GRASP Seminars

- [2022](https://github.com/yuwei-wu/RoboPhD/blob/main/GRASP_Seminars/2022.md)

- [2023](https://github.com/yuwei-wu/RoboPhD/blob/main/GRASP_Seminars/2023.md)

## Annual Reviews

- [TLIOS](https://www.tilos.ai/robotics/)
- [C-BRIC](https://engineering.purdue.edu/C-BRIC)

## Labs

These are the robotic labs I pay special attention on.

- [MIT-ACL](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/MIT-ACL.md)
- [MIT-LIDS](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/MIT-LIDS.md)
- [MIT-RLG](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/MIT-RLG.md)

- [ZJU-FAST Lab](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/ZJU-Fast.md)
- [Upenn-Kumar Lab](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/Upenn-Kumar.md)
- [TU Delft-Autonomous Multi-Robots Laboratory](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/TuDelft-Alonso-Mora.md)
- [HKUST-Aerial Robotics Group](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/Hkust-Shen.md)
- [UZH-RPG](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/UZH-RPG.md)
- [UCSD-ERL](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/UCSD-ERL.md)
- [SNU-LARR](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/SNU-LARR.md)
- [UCB-HiPeR Lab](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/UCB-HiPeR.md)
- [CMU-Air Lab](https://github.com/yuwei-wu/RoboPhD/blob/main/Labs/CMU-Air.md)


## Book Resources

- [Robotics](https://github.com/yuwei-wu/RoboPhD/blob/main/Books/robo.md)

- [Optimization](https://github.com/yuwei-wu/RoboPhD/blob/main/Books/optimization.md)

- [Learning](https://github.com/yuwei-wu/RoboPhD/blob/main/Books/learning.md)

- [CS related](https://github.com/yuwei-wu/RoboPhD/blob/main/Books/cs.md)


## Guidelines

- [Instructions to PhD students by Prof.Dimitris Papadias](https://cse.hkust.edu.hk/~dimitris/Instructions%20for%20PhD%20Students.pdf)

- [Awesome tips for reseach](https://github.com/jbhuang0604/awesome-tips)

- [Ten simple rules for structuring papers](https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1005619)

- [Novelty in Science: A guide to reviewers](https://medium.com/@black_51980/novelty-in-science-8f1fd1a0a143)

- [How to Write Mathmatics](https://entropiesschool.sciencesconf.org/data/How_to_Write_Mathematics.pdf)

## Templates

- [Review and Response Letters](https://github.com/mschroen/review_response_letter)

- [Best README](https://github.com/othneildrew/Best-README-Template)

- [Latex book](https://github.com/amberj/latex-book-template)

## Knowledge Notes

- [The Art of Linear Algebra](https://github.com/kenjihiranabe/The-Art-of-Linear-Algebra)

- [Autonomous Racing Literature](https://github.com/JohannesBetz/AutonomousRacing_Literature)

- [PhD Bibliography on Optimal Control, Reinforcement Learning and Motion Planning](https://github.com/eleurent/phd-bibliography)

- [Deep Implicit Layers](http://implicit-layers-tutorial.org/)


## Robo Tools

### (0) General tools

- [Science Plots](https://github.com/garrettj403/SciencePlots)

- [rosbag_fancy](https://github.com/xqms/rosbag_fancy)

### (1) Solvers:


- [ACADO Toolkit](https://github.com/acado/acado)

  - Use: automatic control and dynamic optimization. It can solve MPC, but has some limits
  - License: open source
  - Interface: C++, with MATLAB

- [CasADi](https://github.com/casadi/casadi)

  - Use: nonlinear optimization and algorithmic differentiation
  - License: open source
  - Interface: C++, Python or Matlab/Octave

- [FORCES PRO](https://www.embotech.com/products/forcespro/overview/)
  - Use: code generator for optimization solver, very useful to solve nonlinear MPC
  - License:  [Academic Licenses](https://www.embotech.com/products/forcespro/licensing/)
  - Interface: C++, Python or Matlab /Simulink interface
  - Some examples: https://github.com/embotech/forcesnlp-examples
  
- [Ceres Solver](https://github.com/ceres-solver/ceres-solver)

  - Use: non-linear Least Squares with bounds constraints/ unconstrained optimization 
  - License: open source
  - Interface: C++ library
  
- [SeDuMi](https://github.com/sqlp/sedumi)

  - Use: linear/quadratic/semidefinite solver
  - License: open source
  - Interface: Matlab/Octave
  
- [Ensmallen](https://github.com/mlpack/ensmallen)
  - Use: non-linear numerical optimization
  - License: open source
  - Interface: C++ library
  
- [HiGHS](https://github.com/ERGO-Code/HiGHS)

  - Use: large scale sparse linear programming
  - License: open source
  - Interface: C, C#, FORTRAN, Julia and Python

- [OR-Tools](https://github.com/google/or-tools)

  - Use: Google Optimization Tools
  - License: open source
  - Interface: C++, but also provide wrappers in Python, C# and Java

- [CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio)

  - Use: IBM optimization studio
  - License: have Free Edition

- [Drake](https://drake.mit.edu/)
  - Use: robotic toolbox, can solve optimizations, systems modeling, and etc.
  - License: open source
  - Interface: C++, python
  - Some examples: https://github.com/RobotLocomotion/drake-external-examples

- [Crocoddyl](https://github.com/loco-3d/crocoddyl)

- [YALMIP](https://yalmip.github.io/)

- [Mosek](https://www.mosek.com/)
  - Use: some types of optimizations. conic, QP, SDP...
  - License: [Academic Licenses](https://www.mosek.com/products/academic-licenses/)
  - Interface: C++, C, python, Matlab
  - Tutorials: https://github.com/MOSEK/Tutorials
  
- [OOQP](https://pages.cs.wisc.edu/~swright/ooqp/)

  - Use: QP
  - License: [MA27 from the HSL Archive](https://www.hsl.rl.ac.uk/download/MA27/1.0.0/a/)
  - Interface: object-oriented C++ package

- [Operator Splitting QP Solver (OSQP)](https://github.com/osqp/osqp)

- [Gurobi](https://www.gurobi.com/products/gurobi-optimizer/)
  - Use: LP, QP and MIP (MILP, MIQP, and MIQCP) 
  - License: [Academic Licenses](https://www.gurobi.com/academia/academic-program-and-licenses/)
  - Interface: C++, C, Python, matlab, R...
  
- [Embedded Conic Solver (ECOS)](https://github.com/embotech/ecos)

  - Use: for convex second-order cone programs (SOCPs) 
  - License: open source
  - Interface: C, Python, Julia, R, Matlab
  
### (2) Simulations: 

- [Webots](https://github.com/cyberbotics/webots)

- [MuJoCo Physics](https://github.com/deepmind/mujoco)

- [Unreal Engine](https://github.com/EpicGames/UnrealEngine)


## Some Famous Planning/Control Repo


### (1) Planner

- [Fast Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)


- [Teach-Repeat-Replan (Autonomous Drone Race)](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan)

- [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)

- [Bilevel Planner](https://github.com/OxDuke/Bilevel-Planner)

- [GPMP2](https://github.com/gtrll/gpmp2)

#### UPenn-MRSL

- [MRSL Motion Primitive Library](https://github.com/sikang/mpl_ros)

#### MIT-ACL

- [FASTER: Fast and Safe Trajectory Planner for Navigation in Unknown Environments](https://github.com/mit-acl/faster)


- [MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](https://github.com/mit-acl/mader)


### (2) Multi-agent


- [multi-robot-trajectory-planning](https://github.com/whoenig/multi-robot-trajectory-planning)

- [Planner using Linear Safe Corridor](https://github.com/qwerty35/lsc_planner)


### MPC-based

- [Model Predictive Contouring Controller (MPCC)](https://github.com/alexliniger/MPCC)

- [Data-Driven MPC for Quadrotors](https://github.com/uzh-rpg/data_driven_mpc)

- [Policy Search for Model Predictive Control with Application to Agile Drone Flight](https://github.com/uzh-rpg/high_mpc)

- [Model Predictive Control for Multi-MAV Collision Avoidance in Dynamic Environments](https://github.com/tud-amr/mrca-mav)


### Optimization

- [DC3: A learning method for optimization with hard constraints](https://github.com/locuslab/DC3)
- [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)


### Map representation

- [Voxblox](https://github.com/ethz-asl/voxblox)

- [Grid Map](https://github.com/ANYbotics/grid_map)

- [OctoMap](https://github.com/OctoMap/octomap)


