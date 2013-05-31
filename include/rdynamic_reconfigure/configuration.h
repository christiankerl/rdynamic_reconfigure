
#include <ros/ros.h>
#include <dynamic_reconfigure/config_tools.h>
#include <dynamic_reconfigure/ConfigDescription.h>

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

namespace rdynamic_reconfigure
{

template<typename T>
class NewParam;

template<typename T>
class ParamRef;

class Configuration;

class ConfigBuilder;

template<typename T>
class ParamDescription
{
public:
  template<typename NewParamT>
  friend class NewParam;

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

  T value() const
  {
    return value_;
  }

  void value(const T& v)
  {
    value_ = v;
  }

  T min() const
  {
    return min_;
  }

  T max() const
  {
    return max_;
  }

  T dflt() const
  {
    return default_;
  }
private:
  std::string name_, description_;
  T value_, min_, max_, default_;

  bool changed_;
};

// ---------------------------- traits ----------------------------
template<typename Type>
struct TypeTraits
{
};

template<>
struct TypeTraits<int>
{
  static const bool IsIntervalType = true;
  static const char* Name() { return "int"; }
  static int DefaultValue() { return 0; }
  static int DefaultMin() { return std::numeric_limits<int>::min(); }
  static int DefaultMax() { return std::numeric_limits<int>::max(); }
  typedef dynamic_reconfigure::IntParameter NativeType;
};

template<>
struct TypeTraits<double>
{
  static const bool IsIntervalType = true;
  static const char* Name() { return "double"; }
  static double DefaultValue() { return 0.0; }
  static double DefaultMin() { return std::numeric_limits<double>::min(); }
  static double DefaultMax() { return std::numeric_limits<double>::max(); }
  typedef dynamic_reconfigure::DoubleParameter NativeType;
};

template<>
struct TypeTraits<std::string>
{
  static const bool IsIntervalType = false;
  static const char* Name() { return "str"; }
  static std::string DefaultValue() { return ""; }
  static std::string DefaultMin() { return DefaultValue(); }
  static std::string DefaultMax() { return DefaultValue(); }
  typedef dynamic_reconfigure::StrParameter NativeType;
};

template<>
struct TypeTraits<bool>
{
  static const bool IsIntervalType = false;
  static const char* Name() { return "bool"; }
  static bool DefaultValue() { return false; }
  static bool DefaultMin() { return DefaultValue(); }
  static bool DefaultMax() { return DefaultValue(); }
  typedef dynamic_reconfigure::BoolParameter NativeType;
};

class Configuration
{
public:
  friend class ConfigBuilder;

  Configuration();
  virtual ~Configuration();

  dynamic_reconfigure::ConfigDescription getDescriptionMessage()
  {
    dynamic_reconfigure::ConfigDescription description;

    defaultToMessage(description.dflt);
    minToMessage(description.min);
    maxToMessage(description.max);

    addParamDescriptionMessagesToGroup(ints_, default_grp_);
    addParamDescriptionMessagesToGroup(doubles_, default_grp_);
    addParamDescriptionMessagesToGroup(strs_, default_grp_);
    addParamDescriptionMessagesToGroup(bools_, default_grp_);

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
    setDefault(ints_);
    setDefault(doubles_);
    setDefault(strs_);
    setDefault(bools_);
  }

  void setChanged(bool changed)
  {
    setChanged(ints_, changed);
    setChanged(doubles_, changed);
    setChanged(strs_, changed);
    setChanged(bools_, changed);
  }

  void fromServer(ros::NodeHandle& nh)
  {
    fromServer(ints_, nh);
    fromServer(doubles_, nh);
    fromServer(strs_, nh);
    fromServer(bools_, nh);
  }

  void toServer(ros::NodeHandle& nh)
  {
    toServer(ints_, nh);
    toServer(doubles_, nh);
    toServer(strs_, nh);
    toServer(bools_, nh);
  }

  void fromMessage(dynamic_reconfigure::Config& msg)
  {
    fromMessage(ints_, msg.ints);
    fromMessage(doubles_, msg.doubles);
    fromMessage(strs_, msg.strs);
    fromMessage(bools_, msg.bools);
  }

  void toMessage(dynamic_reconfigure::Config& msg)
  {
    valueToMessage(msg);
  }

  void clamp()
  {
    clamp(ints_);
    clamp(doubles_);
  }
private:
  dynamic_reconfigure::Group default_grp_;
  dynamic_reconfigure::GroupState default_grp_state_;

  std::map<std::string, ParamDescription<int>* > ints_;
  std::map<std::string, ParamDescription<double>* > doubles_;
  std::map<std::string, ParamDescription<std::string>* > strs_;
  std::map<std::string, ParamDescription<bool>* > bools_;

  template<typename T>
  void toServer(const std::map<std::string, ParamDescription<T>* >& values, ros::NodeHandle& nh)
  {
    typedef std::map<std::string, ParamDescription<T>* > ValueMap;

    for(typename ValueMap::const_iterator it = values.begin(); it != values.end(); ++it)
    {
      nh.setParam(it->first, it->second->value());
    }
  }

  template<typename T>
  void fromServer(std::map<std::string, ParamDescription<T>* >& values, ros::NodeHandle& nh)
  {
    typedef std::map<std::string, ParamDescription<T>* > ValueMap;

    for(typename ValueMap::iterator it = values.begin(); it != values.end(); ++it)
    {
      T v;
      nh.getParam(it->first, v);
      it->second->value(v);
    }
  }

  void valueToMessage(dynamic_reconfigure::Config& msg)
  {
    toMessage<int, &ParamDescription<int>::value>(ints_, msg);
    toMessage<double, &ParamDescription<double>::value>(doubles_, msg);
    toMessage<std::string, &ParamDescription<std::string>::value>(strs_, msg);
    toMessage<bool, &ParamDescription<bool>::value>(bools_, msg);
  }

  void defaultToMessage(dynamic_reconfigure::Config& msg)
  {
    toMessage<int, &ParamDescription<int>::dflt>(ints_, msg);
    toMessage<double, &ParamDescription<double>::dflt>(doubles_, msg);
    toMessage<std::string, &ParamDescription<std::string>::dflt>(strs_, msg);
    toMessage<bool, &ParamDescription<bool>::dflt>(bools_, msg);
  }

  void minToMessage(dynamic_reconfigure::Config& msg)
  {
    toMessage<int, &ParamDescription<int>::min>(ints_, msg);
    toMessage<double, &ParamDescription<double>::min>(doubles_, msg);
    toMessage<std::string, &ParamDescription<std::string>::min>(strs_, msg);
    toMessage<bool, &ParamDescription<bool>::min>(bools_, msg);
  }

  void maxToMessage(dynamic_reconfigure::Config& msg)
  {
    toMessage<int, &ParamDescription<int>::max>(ints_, msg);
    toMessage<double, &ParamDescription<double>::max>(doubles_, msg);
    toMessage<std::string, &ParamDescription<std::string>::max>(strs_, msg);
    toMessage<bool, &ParamDescription<bool>::max>(bools_, msg);
  }

  template<typename T, T (ParamDescription<T>::*Field)() const>
  void toMessage(const std::map<std::string, ParamDescription<T> *>& values, dynamic_reconfigure::Config& msg)
  {
    typedef std::map<std::string, ParamDescription<T>* > ValueMap;

    for(typename ValueMap::const_iterator it = values.begin(); it != values.end(); ++it)
    {
      dynamic_reconfigure::ConfigTools::appendParameter(msg, it->first, (it->second->*Field)());
    }
  }

  template<typename T>
  void fromMessage(std::map<std::string, ParamDescription<T>* >& values, const std::vector<typename TypeTraits<T>::NativeType >& msg)
  {
    typedef std::map<std::string, ParamDescription<T>* > ValueMap;
    typedef std::vector<typename TypeTraits<T>::NativeType > ValueVector;

    for(typename ValueVector::const_iterator it = msg.begin(); it != msg.end(); ++it)
    {
      typename ValueMap::mapped_type& v = values[it->name];
      v->changed(v->value() != it->value);
      v->value(it->value);
    }
  }

  template<typename T>
  void addParamDescriptionMessagesToGroup(const std::map<std::string, ParamDescription<T> *>& values, dynamic_reconfigure::Group& msg)
  {
    typedef std::map<std::string, ParamDescription<T>* > ValueMap;

    dynamic_reconfigure::ParamDescription pdm;
    pdm.type = TypeTraits<T>::Name();
    pdm.level = 0;

    for(typename ValueMap::const_iterator it = values.begin(); it != values.end(); ++it)
    {
      pdm.name = it->second->name();
      pdm.description = it->second->description();

      msg.parameters.push_back(pdm);
    }
  }

  template<typename T>
  void setDefault(const std::map<std::string, ParamDescription<T> *>& values)
  {
    typedef std::map<std::string, ParamDescription<T>* > ValueMap;

    for(typename ValueMap::const_iterator it = values.begin(); it != values.end(); ++it)
    {
      it->second->value(it->second->dflt());
    }
  }

  template<typename T>
  void setChanged(const std::map<std::string, ParamDescription<T> *>& values, bool changed)
  {
    typedef std::map<std::string, ParamDescription<T>* > ValueMap;

    for(typename ValueMap::const_iterator it = values.begin(); it != values.end(); ++it)
    {
      it->second->changed(changed);
    }
  }

  template<typename T>
  void clamp(const std::map<std::string, ParamDescription<T> *>& values)
  {
    typedef std::map<std::string, ParamDescription<T>* > ValueMap;

    for(typename ValueMap::const_iterator it = values.begin(); it != values.end(); ++it)
    {
      if(it->second->value() < it->second->min())
      {
        it->second->value(it->second->min());
      }
      if(it->second->value() > it->second->max())
      {
        it->second->value(it->second->max());
      }
    }
  }

  void addParam(ParamDescription<int>* p)
  {
    ints_.insert(std::make_pair(p->name(), p));
  }

  void addParam(ParamDescription<double>* p)
  {
    doubles_.insert(std::make_pair(p->name(), p));
  }

  void addParam(ParamDescription<std::string>* p)
  {
    strs_.insert(std::make_pair(p->name(), p));
  }

  void addParam(ParamDescription<bool>* p)
  {
    bools_.insert(std::make_pair(p->name(), p));
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

  const ParamDescription<T>& param() const
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

  void bind(ParamDescription<T>* p)
  {
    param_ = p;
  }
private:
  ParamDescription<T>* param_;

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
  NewParam(std::string name, ParamRef<T>& param) :
    param_(new ParamDescription<T>())
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

  ParamDescription<T>* param() const
  {
    return param_;
  }
protected:
  ParamDescription<T>* param_;

  virtual void setIntervalInternal(const T& min, const T& max)
  {
    param_->min_ = min;
    param_->max_ = max;
  }
};

class ConfigBuilder
{
public:
  ConfigBuilder(Configuration& configuration) :
    cfg_(configuration)
  {
  }

  void newGroup(const std::string& name)
  {
  }

  template<typename T>
  ConfigBuilder& add(const NewParam<T>& p)
  {
    cfg_.addParam(p.param());

    return *this;
  }
private:
  Configuration& cfg_;

};

} /* namespace rdynamic_reconfigure */
#endif /* CONFIGURATION_H_ */
