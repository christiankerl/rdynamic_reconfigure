/*
 * configuration.cpp
 *
 *  Created on: May 16, 2013
 *      Author: christiankerl
 */

#include <rdynamic_reconfigure/configuration.h>

namespace rdynamic_reconfigure
{

Configuration::Configuration()
{
  default_grp_.id = 0;
  default_grp_.parent = 0;
  default_grp_.name = "Default";
  default_grp_.type = "";

  default_grp_state_.name = default_grp_.name;
  default_grp_state_.id = default_grp_.id;
  default_grp_state_.parent = default_grp_.parent;
  default_grp_state_.state = 1;
}

Configuration::~Configuration()
{
}

} /* namespace rdynamic_reconfigure */
