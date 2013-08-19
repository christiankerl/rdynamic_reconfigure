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

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/config_tools.h>
#include <dynamic_reconfigure/ConfigDescription.h>

#include <rdynamic_reconfigure/traits.h>

namespace rdynamic_reconfigure
{

template<typename T>
class NewParam;

class AbstractParam
{
public:
  virtual ~AbstractParam() {}

  const std::string& name() const
  {
    return name_;
  }

  const std::string& description() const
  {
    return description_;
  }

  bool changed() const
  {
    return changed_;
  }

  void changed(bool changed)
  {
    changed_ = changed;
  }

  virtual void setDefault() = 0;

  virtual void toParamDescriptionMessage(dynamic_reconfigure::ParamDescription& msg) = 0;

  virtual void toLimitsMessage(dynamic_reconfigure::Config& dflt, dynamic_reconfigure::Config& min, dynamic_reconfigure::Config& max) = 0;

  virtual void toMessage(dynamic_reconfigure::Config& msg) = 0;

  virtual void fromMessage(const dynamic_reconfigure::Config& msg) = 0;

  virtual void toServer(ros::NodeHandle& nh) = 0;

  virtual void fromServer(const ros::NodeHandle& nh) = 0;

  virtual void clamp() = 0;
protected:
  std::string name_, description_;
  bool changed_;
};

template<typename T>
class Param : public AbstractParam
{
public:
  template<typename NewParamT>
  friend class NewParam;

  typedef T Type;
  typedef TypeTraits<T> Trait;

  virtual ~Param() {}

  T value() const
  {
    return value_;
  }

  void value(const T& v)
  {
    value_ = v;
  }

  virtual void setDefault()
  {
    value_ = default_;
  }

  virtual void toParamDescriptionMessage(dynamic_reconfigure::ParamDescription& msg)
  {
    msg.type = Trait::Name();
    msg.level = 0;
    msg.name = name_;
    msg.description = description_;
  }

  virtual void toLimitsMessage(dynamic_reconfigure::Config& dflt, dynamic_reconfigure::Config& min, dynamic_reconfigure::Config& max)
  {
    dynamic_reconfigure::ConfigTools::appendParameter(dflt, name_, default_);
    dynamic_reconfigure::ConfigTools::appendParameter(min, name_, min_);
    dynamic_reconfigure::ConfigTools::appendParameter(max, name_, max_);
  }

  virtual void toMessage(dynamic_reconfigure::Config& msg)
  {
    dynamic_reconfigure::ConfigTools::appendParameter(msg, name_, value_);
  }

  virtual void fromMessage(const dynamic_reconfigure::Config& msg)
  {
    T new_value = value_;
    if(dynamic_reconfigure::ConfigTools::getParameter(msg, name_, new_value))
    {
      changed_ = value_ != new_value;
      value_ = new_value;
    }
  }

  virtual void toServer(ros::NodeHandle& nh)
  {
    nh.setParam(name_, value_);
  }

  virtual void fromServer(const ros::NodeHandle& nh)
  {
    nh.getParam(name_, value_);
  }

  virtual void clamp()
  {
    if(Trait::IsIntervalType)
    {
      value_ = std::min(max_, std::max(value_, min_));
    }
  }

protected:
  T value_, min_, max_, default_;
};

// --------------------  enums  ----------------------------------
class EnumParam;

class NewEnumParam;

class EnumConstant
{
public:
  friend class EnumParam;

  int id() const
  {
    return id_;
  }
private:
  int id_;
};

class EnumParam : public Param<int>
{
public:
  friend class NewEnumParam;

  EnumParam() :
    next_id_(1)
  {
    default_ = next_id_;
    min_ = next_id_;
  }

  virtual ~EnumParam() {}

  virtual void toParamDescriptionMessage(dynamic_reconfigure::ParamDescription& msg)
  {
    std::string separator = "";

    std::stringstream edit_method;
    edit_method << "{'enum_description': '" << name() << "', 'enum': [";

    for(EntryVector::const_iterator it = entries_.begin(); it != entries_.end(); ++it)
    {
      edit_method
        << separator << "{"
        << "'name': " << "'" << it->name << "',"
        << "'description': " << "'',"
        << "'value': " << it->id << ","
        << "'type': 'int'"
        << "}";

      separator = ", ";
    }

    edit_method << "]}";

    Param<int>::toParamDescriptionMessage(msg);
    msg.edit_method = edit_method.str();
  }

  void add(const std::string& name, EnumConstant& value)
  {
    value.id_ = next_id_++;
    max_ = value.id_;

    EnumEntry e;
    e.id = value.id_;
    e.name = name;
    entries_.push_back(e);
  }
private:
  struct EnumEntry
  {
    std::string name;
    int id;
  };

  typedef std::vector<EnumEntry> EntryVector;

  int next_id_;
  EntryVector entries_;
};

class Configuration
{
public:
  Configuration();
  virtual ~Configuration();

  dynamic_reconfigure::ConfigDescription getDescriptionMessage()
  {
    dynamic_reconfigure::ConfigDescription description;

    for(ParamMap::iterator it = params_.begin(); it != params_.end(); ++it)
    {
      it->second->toLimitsMessage(description.dflt, description.min, description.max);

      dynamic_reconfigure::ParamDescription pdm;
      it->second->toParamDescriptionMessage(pdm);
      default_grp_.parameters.push_back(pdm);
    }

    description.groups.push_back(default_grp_);
    description.dflt.groups.push_back(default_grp_state_);
    description.min.groups.push_back(default_grp_state_);
    description.max.groups.push_back(default_grp_state_);

    return description;
  }

  dynamic_reconfigure::Config getMessage()
  {
    dynamic_reconfigure::Config msg;
    toMessage(msg);
    msg.groups.push_back(default_grp_state_);

    return msg;
  }

  void setDefault()
  {
    for(ParamMap::iterator it = params_.begin(); it != params_.end(); ++it)
      it->second->setDefault();
  }

  void setChanged(bool changed)
  {
    for(ParamMap::iterator it = params_.begin(); it != params_.end(); ++it)
      it->second->changed(changed);
  }

  void fromServer(const ros::NodeHandle& nh)
  {
    for(ParamMap::iterator it = params_.begin(); it != params_.end(); ++it)
      it->second->fromServer(nh);
  }

  void toServer(ros::NodeHandle& nh)
  {
    for(ParamMap::iterator it = params_.begin(); it != params_.end(); ++it)
      it->second->toServer(nh);
  }

  void fromMessage(const dynamic_reconfigure::Config& msg)
  {
    for(ParamMap::iterator it = params_.begin(); it != params_.end(); ++it)
      it->second->fromMessage(msg);
  }

  void toMessage(dynamic_reconfigure::Config& msg)
  {
    for(ParamMap::iterator it = params_.begin(); it != params_.end(); ++it)
      it->second->toMessage(msg);
  }

  void clamp()
  {
    for(ParamMap::iterator it = params_.begin(); it != params_.end(); ++it)
      it->second->clamp();
  }

  template<typename T>
  Configuration& add(const T& p)
  {
    addParam(p.param());

    return *this;
  }

private:
  dynamic_reconfigure::Group default_grp_;
  dynamic_reconfigure::GroupState default_grp_state_;

  typedef std::map<std::string, AbstractParam*> ParamMap;

  ParamMap params_;

  void addParam(AbstractParam* p)
  {
    params_.insert(std::make_pair(p->name(), p));
  }
};

class UnboundParamRefException : public std::exception
{
public:
  UnboundParamRefException() throw() {}
  virtual ~UnboundParamRefException() throw() {}
};

template<typename T>
class ParamRef
{
public:
  ParamRef() : param_(0)
  {
  }

  bool changed() const
  {
    return bound() ? param_->changed() : false;
  }

  bool bound() const
  {
    return param_ != 0;
  }

  const Param<T>& param() const
  {
    assertIsBound();

    return *param_;
  }

  void operator =(const T& v)
  {
    assertIsBound();

    param_->value(v);
  }

  operator T() const
  {
    assertIsBound();

    return param_->value();
  }

  void bind(Param<T>* p)
  {
    param_ = p;
  }
private:
  Param<T>* param_;

  void assertIsBound() const
  {
    if(!bound())
      throw UnboundParamRefException();
  }
};


// ---------------------------- interval concept ----------------------------
template<typename Derived, typename T, bool IsIntervalType>
struct ParamBuilderIntervalConcept
{
protected:
  virtual void setIntervalInternal(const T& min, const T& max) = 0;
};

template<typename Derived, typename T>
struct ParamBuilderIntervalConcept<Derived, T, true>
{
public:
  Derived& setInterval(const T& min, const T& max)
  {
    setIntervalInternal(min, max);
    return *static_cast<Derived*>(this);
  }
protected:
  virtual void setIntervalInternal(const T& min, const T& max) = 0;
};


template<typename T>
class NewParam : public ParamBuilderIntervalConcept<NewParam<T>, T, TypeTraits<T>::IsIntervalType>
{
public:
  NewParam(const std::string& name, ParamRef<T>& param) :
    param_(new Param<T>())
  {
    param.bind(param_);

    param_->name_ = name;
    param_->default_ = TypeTraits<T>::DefaultValue();
    setIntervalInternal(TypeTraits<T>::DefaultMin(), TypeTraits<T>::DefaultMax());
  }

  NewParam<T>& setDescription(const std::string& description)
  {
    param_->description_ = description;
    return *this;
  }

  NewParam<T>& setDefault(const T& value)
  {
    param_->default_ = value;
    return *this;
  }

  Param<T>* param() const
  {
    return param_;
  }
protected:
  Param<T>* param_;

  virtual void setIntervalInternal(const T& min, const T& max)
  {
    param_->min_ = min;
    param_->max_ = max;
  }
};

class NewEnumParam
{
public:
  NewEnumParam(const std::string& name, ParamRef<int>& param) :
    param_(new EnumParam())
  {
    param.bind(param_);
    param_->name_ = name;
  }

  NewEnumParam& setDescription(const std::string& description)
  {
    param_->description_ = description;
    return *this;
  }

  NewEnumParam& setDefault(const EnumConstant& value)
  {
    param_->default_ = value.id();
    return *this;
  }

  NewEnumParam& add(const std::string& name, EnumConstant& value)
  {
    param_->add(name, value);
    return *this;
  }

  EnumParam* param() const
  {
    return param_;
  }
private:
  EnumParam* param_;
};


} /* namespace rdynamic_reconfigure */
#endif /* CONFIGURATION_H_ */
