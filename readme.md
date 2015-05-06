# OpenFleet Toolkit

The purpose of this project is to develop motion planning algorithms to enable autonomous imaging and exploration for fleets of small satellites. Sending humans into space remains extremely expensive. However, the cost per kilogram of launching cargo into space is following a Moore’s law decrease, which has fomented a huge increase of privatized small (1-10kg) satellites being launched into orbit. We think that robotic motion planning will enable new possibilities in areas of space imaging, communications, and exploration.
 
Our starting point was Lavalle and Kuffner’s Randomized Kinodynamic Planning paper [1], in which they applied their RRT algorithm to motion planning for hovercrafts in dynamic state space. Since we want our models to be based on physically realizable prototypes, our team spoke with engineers from two NYC based space companies based in NYC: Ragnarok and Honeybee Robotics, and with a managing director of Accion Propulsion. These conversations and product specs ([2], [3], and [4]) validated that the control systems on state of the art small satellite platforms could support very precise force and torque applications as modeled in Lavalle et al’s work.  

Our key contribution is generalizing LaValle et al’s motion planning algorithms to fleets of satellites in order to accomplish synergistic outcomes like stereo imaging. In our generalization, we want to validate whether the increased degrees of freedom involved with fleet planning can come with a discounted computational cost. The key to making this work is developing adequate inter-fleet collision checking. We also design, run and render simulations of missions based on real world models like the International Space Station, the US Space Shuttle, and RQ36 and ‘Shuttle Island’ asteroids.  

## Participants

Andre Cunha

Chris Cleveland

Marcello Salomao
