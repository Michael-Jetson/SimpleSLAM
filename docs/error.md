# 报错实录与原因分析+学习记录

1. /usr/include/c++/11/bits/alloc_traits.h:518:28: error: no matching function for call to ‘construct_at(SimpleSlam::IMUSubModule*&, const char [14])’
  518 |           std::construct_at(__p, std::forward<_Args>(__args)...);
  分析原因：有虚函数没有重写，导致构造出错

2.sensor_io.h:73:10: error: ‘bool SimpleSlam::IMUSubModule::init(const SimpleSlam::System*, const SimpleSlam::Module*)’ cannot be overloaded with ‘bool SimpleSlam::IMUSubModule::init(const SimpleSlam::System*, const SimpleSlam::Module*)’
  分析原因：复制粘贴的时候声明了两次一模一样的init函数