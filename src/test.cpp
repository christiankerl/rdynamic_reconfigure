/**
 * Copyright (c) 2013 Christian Kerl <christian.kerl@in.tum.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

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

  config
  .add(NewParam<int>("test_int", t.v0).setDescription("test parameter").setInterval(0, 10))
  .add(NewParam<double>("test_double", t.v1).setDescription("test parameter").setInterval(-10.0, 10.0))
  .add(NewParam<bool>("test_bool", t.v2).setDescription("test parameter").setDefault(true))
  .add(NewParam<std::string>("test_string", t.v3).setDescription("test parameter").setDefault("hello"));

  rdynamic_reconfigure::Server server(nh, config);
  server.setCallback(boost::bind(&TestNode::onConfig, &t, _1));

  ros::spin();

  return 0;
}
