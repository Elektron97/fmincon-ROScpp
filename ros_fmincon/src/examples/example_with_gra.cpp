/****************************************
 * ros_fmincon Example with Gradient    *
 ****************************************/
// --- Includes --- //
#include <iostream>
#include <cmath>
// fmincon library
#include "ros_fmincon/sci_arma.h"
// ros library
#include "ros/ros.h"

// --- Defines --- //
#define NODE_FREQUENCY 10.0 //[Hz]

// --- Global Variables --- //
const std::string node_name = "example_with_grad";

int main(int argc, char** argv) 
{
    // Init Ros Node
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_obj;

    // Rate Obj
    ros::Rate loop_rate(NODE_FREQUENCY);

    //define a obj_fun
    obj_fun f=[](vec& x)-> double {
        return pow(1-0.1*x(0),2)+100*pow(x(1)-0.5*x(0)*x(0),2)
               +pow(1-0.1*x(2),2)+100*pow(x(3)-0.5*x(2)*x(2),2);
    };

    //define the gradient
    gradient g=[](vec& x)->vec{
        vec result= {-0.2*(1-0.1*x(0)) - 200*x(0)*(x(1)-0.5*x(0)*x(0)),
                     200*(x(1)-0.5*x(0)*x(0)),
                     -0.2*(1-0.1*x(2)) - 200*x(2)*(x(3)-0.5*x(2)*x(2)),
                     200*(x(3)-0.5*x(2)*x(2))};
        return result;
    };

    options opt;
    opt.gra=g;
    opt.enable_self_defined_gra= true;
    opt.max_ite=5000;
    vec x0= {0,0,0,0};
    auto result1= sci_arma::fmincon(f, x0,opt);

    while(ros::ok())
    {
        ROS_INFO("Optimization Result:");
        std::cout<<result1<<endl;
        loop_rate.sleep();
    }

    return 0;
}