# Function fmincon() with ROS C++
This repository is a fork of the [fmincon with C++](https://github.com/Hangcil/fmincon-with-Cpp). In this fork, I re-implemented the library in a ros pkg, so you can use this library in your ros projects. The original repository only support **nonlinear inequality constraints** and not **nonlinear equality constraints** :( .
## Algorithms
1. BFGS (default when the constraint is empty)
2. Powell 
3. modified Powell in Sargent form (default when BFGS fails)
4. Rosenbrock
5. mutiplier (it will be used with one of the methods above when there are constraints)
6. exterior-point (legacy)

## Examples
Just rewriting the examples of the original repository such as a simple node that print the solutions. You can find these examples here:
- [Example with Gradient](ros_fmincon/src/examples/example_with_gra.cpp)
- [Example without Gradient](ros_fmincon/src/examples/example_without_gra.cpp)

## Ubuntu Branch
If you are interested only in using this library in Ubuntu, I also adapt the make file in the [ubuntu branch](https://github.com/Elektron97/fmincon-ROScpp/tree/ubuntu).
