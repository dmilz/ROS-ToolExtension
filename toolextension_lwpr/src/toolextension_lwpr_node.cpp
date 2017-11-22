#include <toolextension_lwpr_node.h>


int main(int argc, char **argv)
{
        ros::init(argc, argv, "toolextension_lwpr");
        ros::NodeHandle n;
        ROS_INFO("Starting toolextension_lwpr_node...");
        toolextension_lwpr::Node node(n, "config.cfg");
        node.run();


        return 0;
}



