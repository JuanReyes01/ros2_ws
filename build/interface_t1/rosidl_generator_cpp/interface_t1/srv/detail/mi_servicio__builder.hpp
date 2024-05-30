// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_t1:srv/MiServicio.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__BUILDER_HPP_
#define INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_t1/srv/detail/mi_servicio__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_t1
{

namespace srv
{

namespace builder
{

class Init_MiServicio_Request_ruta
{
public:
  Init_MiServicio_Request_ruta()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface_t1::srv::MiServicio_Request ruta(::interface_t1::srv::MiServicio_Request::_ruta_type arg)
  {
    msg_.ruta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_t1::srv::MiServicio_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_t1::srv::MiServicio_Request>()
{
  return interface_t1::srv::builder::Init_MiServicio_Request_ruta();
}

}  // namespace interface_t1


namespace interface_t1
{

namespace srv
{

namespace builder
{

class Init_MiServicio_Response_confirmacion
{
public:
  Init_MiServicio_Response_confirmacion()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface_t1::srv::MiServicio_Response confirmacion(::interface_t1::srv::MiServicio_Response::_confirmacion_type arg)
  {
    msg_.confirmacion = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_t1::srv::MiServicio_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_t1::srv::MiServicio_Response>()
{
  return interface_t1::srv::builder::Init_MiServicio_Response_confirmacion();
}

}  // namespace interface_t1

#endif  // INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__BUILDER_HPP_
