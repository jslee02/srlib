#===============================================================================
function(srlib_property_add property_name)

  get_property(is_defined GLOBAL PROPERTY ${property_name} DEFINED)

  if(NOT is_defined)
    define_property(GLOBAL PROPERTY ${property_name}
      BRIEF_DOCS "${property_name}"
      FULL_DOCS "Global properties for ${property_name}"
    )
  endif()

  foreach(item ${ARGN})
    set_property(GLOBAL APPEND PROPERTY ${property_name} "${item}")
  endforeach()

endfunction()

#===============================================================================
function(srlib_add_headers)
  foreach(item ${ARGN})
    set_property(GLOBAL APPEND PROPERTY DART_HEADERS ${item})
  endforeach()
endfunction()

#===============================================================================
function(srlib_get_headers output_var)
  get_property(var GLOBAL PROPERTY DART_HEADERS)
  set(${output_var} ${var} PARENT_SCOPE)
endfunction()
