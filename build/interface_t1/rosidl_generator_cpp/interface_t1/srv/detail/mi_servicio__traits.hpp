// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_t1:srv/MiServicio.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__TRAITS_HPP_
#define INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_t1/srv/detail/mi_servicio__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interface_t1
{

namespace srv
{

inline void to_flow_style_yaml(
  const MiServicio_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: ruta
  {
    out << "ruta: ";
    rosidl_generator_traits::value_to_yaml(msg.ruta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MiServicio_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ruta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ruta: ";
    rosidl_generator_traits::value_to_yaml(msg.ruta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MiServicio_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interface_t1

namespace rosidl_generator_traits
{

[[deprecated("use interface_t1::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_t1::srv::MiServicio_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_t1::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_t1::srv::to_yaml() instead")]]
inline std::string to_yaml(const interface_t1::srv::MiServicio_Request & msg)
{
  return interface_t1::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interface_t1::srv::MiServicio_Request>()
{
  return "interface_t1::srv::MiServicio_Request";
}

template<>
inline const char * name<interface_t1::srv::MiServicio_Request>()
{
  return "interface_t1/srv/MiServicio_Request";
}

template<>
struct has_fixed_size<interface_t1::srv::MiServicio_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_t1::srv::MiServicio_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_t1::srv::MiServicio_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interface_t1
{

namespace srv
{

inline void to_flow_style_yaml(
  const MiServicio_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: confirmacion
  {
    out << "confirmacion: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmacion, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MiServicio_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: confirmacion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirmacion: ";
    rosidl_generator_traits::value_to_yaml(msg.confirmacion, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MiServicio_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interface_t1

namespace rosidl_generator_traits
{

[[deprecated("use interface_t1::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_t1::srv::MiServicio_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_t1::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_t1::srv::to_yaml() instead")]]
inline std::string to_yaml(const interface_t1::srv::MiServicio_Response & msg)
{
  return interface_t1::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interface_t1::srv::MiServicio_Response>()
{
  return "interface_t1::srv::MiServicio_Response";
}

template<>
inline const char * name<interface_t1::srv::MiServicio_Response>()
{
  return "interface_t1/srv/MiServicio_Response";
}

template<>
struct has_fixed_size<interface_t1::srv::MiServicio_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface_t1::srv::MiServicio_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface_t1::srv::MiServicio_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface_t1::srv::MiServicio>()
{
  return "interface_t1::srv::MiServicio";
}

template<>
inline const char * name<interface_t1::srv::MiServicio>()
{
  return "interface_t1/srv/MiServicio";
}

template<>
struct has_fixed_size<interface_t1::srv::MiServicio>
  : std::integral_constant<
    bool,
    has_fixed_size<interface_t1::srv::MiServicio_Request>::value &&
    has_fixed_size<interface_t1::srv::MiServicio_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface_t1::srv::MiServicio>
  : std::integral_constant<
    bool,
    has_bounded_size<interface_t1::srv::MiServicio_Request>::value &&
    has_bounded_size<interface_t1::srv::MiServicio_Response>::value
  >
{
};

template<>
struct is_service<interface_t1::srv::MiServicio>
  : std::true_type
{
};

template<>
struct is_service_request<interface_t1::srv::MiServicio_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface_t1::srv::MiServicio_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__TRAITS_HPP_
