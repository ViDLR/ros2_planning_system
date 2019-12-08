![PlanSys2 Logo](/plansys2_docs/plansys2_logo.png)

[![Build Status](https://travis-ci.com/IntelligentRoboticsLabs/ros2_planning_system.svg?branch=master)](https://travis-ci.com/IntelligentRoboticsLabs/ros2_planning_system)


ROS2 Planning System (**plansys2** in short) is a project whose objective is to provide Robotics developers with a reliable, simple, and efficient PDDL-based planning system. It is implemented in ROS2, applying the latest concepts developed in this currently de-facto standard in Robotics.

This project is the result of several years of experience in the development of robotic behaviors using [ROSPlan](https://github.com/KCL-Planning/ROSPlan). ROSPlan has greatly inspired this project. In addition to the migration to ROS2, we contribute to key aspects: ease of use, efficiency, and new tools, such as our terminal.

We hope that this software helps to include planning in more Robotics projects, offering simple and powerful software to generate intelligent behaviors for robots.

We want to invite you to contribute to this Open Source project !!

# Requirements and compilation

This project was initially developed for ROS2 Eloquent. In addition to official packages, plansys2 requires popf, a PDDL plan solver, developed by Marc Hanheide, to which we have contributed to its migration to a ROS2 package.

Before compiling, include popf in your workspace:

```
plansys2_ws/src$ sudo apt-get install flex bison coinor-*
plansys2_ws/src$ git clone https://github.com/LCAS/popf.git
```

Note: The previous URL may change, since Marc has generously decided to transfer ownership of this project, with the aim of revitalizing its development by including it in plansys2

Next, only compile:

```
plansys2_ws/src$ colcon build --symlink-install
```

# Working with Plansys2 and Terminal

Check out this (PDDL domain example](plansys2_examples/plansys2_simple_example/pddl/simple_example.pddl). This is a small example of a tiny domain. It defines the types, predicates, and three actions for making a robot moving taking into account the battery level. It is a very basic example, but it is useful to illustrate this example.

Open a terminal and launch plansys2. We will use a launcher that includes the main planning system launcher, the specific action nodes for this example, and selects the domain:

```
ros2 launch plansys2_simple_example plansys2_simple_example_launch.py
```

Open other terminal and launch the plansys2 terminal:

```
ros2 run plansys2_terminal plansys2_terminal
```

The plansys2 terminal lets us operate directly against the planning system. It is a useful tool, useful to monitorize and developing your application. Usually, many of the next operations should be done inside your nodes. Plansys2 terminal is functional, but there is still too much to improve.

Inside the plansys2 terminal, you can check the domain:

```
> get domain
```

You also can check the current instances in the plan, that now it's void:

```
> get problem instances
```

To get the current predicates:

```
> get problem predicates
```

Now, the problem is void. Let's add some content and recheck it after.

```
> set instance leia robot
> set instance entrance room
> set instance kitchen room
> set instance bedroom room
> set instance dinning room
> set instance bathroom room
> set instance chargingroom room
> set predicate (connected entrance dinning)
> set predicate (connected dinning entrance)
> set predicate (connected dinning kitchen)
> set predicate (connected kitchen dinning)
> set predicate (connected dinning bedroom)
> set predicate (connected bedroom dinning)
> set predicate (connected bathroom bedroom)
> set predicate (connected bedroom bathroom)
> set predicate (connected chargingroom kitchen)
> set predicate (connected kitchen chargingroom)
> set predicate (charging_point_at chargingroom)
> set predicate (battery_low leia)
> set predicate (robot_at leia entrance)
```

Once added content to the problem, verify the content of the problem again:

```
> get problem instances
...
> get problem predicates
```

Lets planify. Add a goal to be achieved:
```
> set goal (and(robot_at leia bathroom))
```

And get the plan. This command will not execute the plan. Only will calculate it:


```
> get plan
plan: 
0 (askcharge leia entrance chargingroom) 5
0.001 (charge leia chargingroom) 5
5.002 (move leia chargingroom kitchen) 5
10.003 (move leia kitchen dinning) 5
15.004 (move leia dinning bedroom) 5
20.005 (move leia bedroom bathroom) 5
```

To run the plan, type:

```
run
```

You will see how the actions (calling to the nodes that implement the actions) are executed.

press Ctrl-D to exit. 


The Plansys2 terminal has many functionalities: Adding/removing instances and predicates, asking for model details (predicates, actions, and types), run actions, getting plans, and running plans.

# Further readings

- Design (under development)
- PDDL basics [1](https://arxiv.org/pdf/1106.4561.pdf), [2](http://www.cs.toronto.edu/~sheila/2542/w09/A1/introtopddl2.pdf) and [3](http://www.cs.toronto.edu/~sheila/384/w11/Assignments/A3/veloso-PDDL_by_Example.pdf)
- API (under development)
- FAQs(under development)

<img src="/plansys2_docs/plansys2_logo_v2.png" alt="drawing" width="200" align="right"/>

