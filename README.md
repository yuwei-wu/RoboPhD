# RoboPhD
Some records of arXiv papers, GRASP seminars, and resources during my PhD period.



## ArXiv Papers

The papers and notes updates weekly, mainly about motion planning

- [2021](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2021.md)

- [2022](https://github.com/yuwei-wu/RoboPhD/blob/main/arXiv/2022.md)


## GRASP Seminars

- [2022](https://github.com/yuwei-wu/RoboPhD/blob/main/GRASP_Seminars/2022.md)

## Work by Labs

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


## Resources

- [Optimization](https://github.com/yuwei-wu/RoboPhD/blob/main/Resources/optimization.md)

- [Learning](https://github.com/yuwei-wu/RoboPhD/blob/main/Resources/learning.md)

- [CS related](https://github.com/yuwei-wu/RoboPhD/blob/main/Resources/cs.md)


## Guidelines

- [Instructions to PhD students by Prof.Dimitris Papadias](https://cse.hkust.edu.hk/~dimitris/Instructions%20for%20PhD%20Students.pdf)

- [Awesome tips for reseach](https://github.com/jbhuang0604/awesome-tips)

- [Ten simple rules for structuring papers](https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1005619)

- [PhD Bibliography on Optimal Control, Reinforcement Learning and Motion Planning](https://github.com/eleurent/phd-bibliography)

- [Autonomous Racing Literature](https://github.com/JohannesBetz/AutonomousRacing_Literature)

- [Novelty in Science: A guide to reviewers](https://medium.com/@black_51980/novelty-in-science-8f1fd1a0a143)

- [Small Unmanned Aircraft: Theory and Practice](https://github.com/randybeard/uavbook)

## Templates

- [Review and Response Letters](https://github.com/mschroen/review_response_letter)

- [Best README](https://github.com/othneildrew/Best-README-Template)


## Robo Tools

#### (0) Total

- [Science Plots](https://github.com/garrettj403/SciencePlots)


#### (1) Solvers:


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
  
#### (2) Simulations: 

- [Webots](https://github.com/cyberbotics/webots)

- [MuJoCo Physics](https://github.com/deepmind/mujoco)

- [Unreal Engine](https://github.com/EpicGames/UnrealEngine)

