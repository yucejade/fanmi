// Copyright (c) 2008-2023 the Urho3D project
// License: MIT

#ifndef URHO3D_API_H
#define URHO3D_API_H

#ifdef URHO3D_STATIC_DEFINE
#  define URHO3D_API
#  define URHO3D_NO_EXPORT
#else
#  ifndef URHO3D_API
#    ifdef Urho3D_EXPORTS
        /* We are building this library */
#      define URHO3D_API 
#    else
        /* We are using this library */
#      define URHO3D_API 
#    endif
#  endif

#  ifndef URHO3D_NO_EXPORT
#    define URHO3D_NO_EXPORT 
#  endif
#endif

#ifndef URHO3D_DEPRECATED
#  define URHO3D_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef URHO3D_DEPRECATED_EXPORT
#  define URHO3D_DEPRECATED_EXPORT URHO3D_API URHO3D_DEPRECATED
#endif

#ifndef URHO3D_DEPRECATED_NO_EXPORT
#  define URHO3D_DEPRECATED_NO_EXPORT URHO3D_NO_EXPORT URHO3D_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef URHO3D_NO_DEPRECATED
#    define URHO3D_NO_DEPRECATED
#  endif
#endif
#define URHO3D_OPENGL

#endif /* URHO3D_API_H */
