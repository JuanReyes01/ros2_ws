# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interface_t1:srv/MiServicio.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MiServicio_Request(type):
    """Metaclass of message 'MiServicio_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interface_t1')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interface_t1.srv.MiServicio_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__mi_servicio__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__mi_servicio__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__mi_servicio__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__mi_servicio__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__mi_servicio__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MiServicio_Request(metaclass=Metaclass_MiServicio_Request):
    """Message class 'MiServicio_Request'."""

    __slots__ = [
        '_ruta',
    ]

    _fields_and_field_types = {
        'ruta': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ruta = kwargs.get('ruta', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.ruta != other.ruta:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ruta(self):
        """Message field 'ruta'."""
        return self._ruta

    @ruta.setter
    def ruta(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ruta' field must be of type 'str'"
        self._ruta = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MiServicio_Response(type):
    """Metaclass of message 'MiServicio_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interface_t1')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interface_t1.srv.MiServicio_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__mi_servicio__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__mi_servicio__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__mi_servicio__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__mi_servicio__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__mi_servicio__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MiServicio_Response(metaclass=Metaclass_MiServicio_Response):
    """Message class 'MiServicio_Response'."""

    __slots__ = [
        '_confirmacion',
    ]

    _fields_and_field_types = {
        'confirmacion': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.confirmacion = kwargs.get('confirmacion', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.confirmacion != other.confirmacion:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def confirmacion(self):
        """Message field 'confirmacion'."""
        return self._confirmacion

    @confirmacion.setter
    def confirmacion(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'confirmacion' field must be of type 'bool'"
        self._confirmacion = value


class Metaclass_MiServicio(type):
    """Metaclass of service 'MiServicio'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interface_t1')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interface_t1.srv.MiServicio')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__mi_servicio

            from interface_t1.srv import _mi_servicio
            if _mi_servicio.Metaclass_MiServicio_Request._TYPE_SUPPORT is None:
                _mi_servicio.Metaclass_MiServicio_Request.__import_type_support__()
            if _mi_servicio.Metaclass_MiServicio_Response._TYPE_SUPPORT is None:
                _mi_servicio.Metaclass_MiServicio_Response.__import_type_support__()


class MiServicio(metaclass=Metaclass_MiServicio):
    from interface_t1.srv._mi_servicio import MiServicio_Request as Request
    from interface_t1.srv._mi_servicio import MiServicio_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
