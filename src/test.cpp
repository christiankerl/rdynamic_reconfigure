
#include <ros/ros.h>
#include <rdynamic_reconfigure/configuration.h>
#include <rdynamic_reconfigure/server.h>

using namespace rdynamic_reconfigure;

struct TestNode
{
  ParamRef<int> v0;
  ParamRef<double> v1;
  ParamRef<bool> v2;
  ParamRef<std::string> v3;

  void onConfig(Configuration& cfg)
  {
    v0 = 1;

    if(v2.changed())
    {
      std::cerr << "hey you toggled me!" << std::endl;
      v2 = false;
    }

    std::cerr << "hello " << v1 << std::endl;
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "rdynamic_reconfigure_example");

  ros::NodeHandle nh("~");

  TestNode t;

  Configuration config;

  ConfigBuilder(config)
  .add(NewParam<int>("test_int", t.v0).setDescription("test parameter").setInterval(0, 10))
  .add(NewParam<double>("test_double", t.v1).setDescription("test parameter").setInterval(-10.0, 10.0))
  .add(NewParam<bool>("test_bool", t.v2).setDescription("test parameter").setDefault(true))
  .add(NewParam<std::string>("test_string", t.v3).setDescription("test parameter").setDefault("hello"));

  rdynamic_reconfigure::Server server(nh, config);
  server.setCallback(boost::bind(&TestNode::onConfig, &t, _1));

  ros::spin();

  return 0;
}
