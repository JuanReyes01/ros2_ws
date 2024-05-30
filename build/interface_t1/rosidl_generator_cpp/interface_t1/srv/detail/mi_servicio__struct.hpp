// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_t1:srv/MiServicio.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__STRUCT_HPP_
#define INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface_t1__srv__MiServicio_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface_t1__srv__MiServicio_Request __declspec(deprecated)
#endif

namespace interface_t1
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MiServicio_Request_
{
  using Type = MiServicio_Request_<ContainerAllocator>;

  explicit MiServicio_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ruta = "";
    }
  }

  explicit MiServicio_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ruta(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ruta = "";
    }
  }

  // field types and members
  using _ruta_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ruta_type ruta;

  // setters for named parameter idiom
  Type & set__ruta(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ruta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_t1::srv::MiServicio_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_t1::srv::MiServicio_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_t1::srv::MiServicio_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_t1::srv::MiServicio_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_t1::srv::MiServicio_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_t1::srv::MiServicio_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_t1::srv::MiServicio_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_t1::srv::MiServicio_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_t1::srv::MiServicio_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_t1::srv::MiServicio_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_t1__srv__MiServicio_Request
    std::shared_ptr<interface_t1::srv::MiServicio_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_t1__srv__MiServicio_Request
    std::shared_ptr<interface_t1::srv::MiServicio_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MiServicio_Request_ & other) const
  {
    if (this->ruta != other.ruta) {
      return false;
    }
    return true;
  }
  bool operator!=(const MiServicio_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MiServicio_Request_

// alias to use template instance with default allocator
using MiServicio_Request =
  interface_t1::srv::MiServicio_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_t1


#ifndef _WIN32
# define DEPRECATED__interface_t1__srv__MiServicio_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface_t1__srv__MiServicio_Response __declspec(deprecated)
#endif

namespace interface_t1
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MiServicio_Response_
{
  using Type = MiServicio_Response_<ContainerAllocator>;

  explicit MiServicio_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confirmacion = false;
    }
  }

  explicit MiServicio_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confirmacion = false;
    }
  }

  // field types and members
  using _confirmacion_type =
    bool;
  _confirmacion_type confirmacion;

  // setters for named parameter idiom
  Type & set__confirmacion(
    const bool & _arg)
  {
    this->confirmacion = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_t1::srv::MiServicio_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_t1::srv::MiServicio_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_t1::srv::MiServicio_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_t1::srv::MiServicio_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_t1::srv::MiServicio_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_t1::srv::MiServicio_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_t1::srv::MiServicio_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_t1::srv::MiServicio_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_t1::srv::MiServicio_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_t1::srv::MiServicio_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_t1__srv__MiServicio_Response
    std::shared_ptr<interface_t1::srv::MiServicio_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_t1__srv__MiServicio_Response
    std::shared_ptr<interface_t1::srv::MiServicio_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MiServicio_Response_ & other) const
  {
    if (this->confirmacion != other.confirmacion) {
      return false;
    }
    return true;
  }
  bool operator!=(const MiServicio_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MiServicio_Response_

// alias to use template instance with default allocator
using MiServicio_Response =
  interface_t1::srv::MiServicio_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_t1

namespace interface_t1
{

namespace srv
{

struct MiServicio
{
  using Request = interface_t1::srv::MiServicio_Request;
  using Response = interface_t1::srv::MiServicio_Response;
};

}  // namespace srv

}  // namespace interface_t1

#endif  // INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__STRUCT_HPP_
