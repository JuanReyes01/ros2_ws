// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_t1:srv/MiServicio.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__STRUCT_H_
#define INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ruta'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MiServicio in the package interface_t1.
typedef struct interface_t1__srv__MiServicio_Request
{
  rosidl_runtime_c__String ruta;
} interface_t1__srv__MiServicio_Request;

// Struct for a sequence of interface_t1__srv__MiServicio_Request.
typedef struct interface_t1__srv__MiServicio_Request__Sequence
{
  interface_t1__srv__MiServicio_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_t1__srv__MiServicio_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MiServicio in the package interface_t1.
typedef struct interface_t1__srv__MiServicio_Response
{
  bool confirmacion;
} interface_t1__srv__MiServicio_Response;

// Struct for a sequence of interface_t1__srv__MiServicio_Response.
typedef struct interface_t1__srv__MiServicio_Response__Sequence
{
  interface_t1__srv__MiServicio_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_t1__srv__MiServicio_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_T1__SRV__DETAIL__MI_SERVICIO__STRUCT_H_
