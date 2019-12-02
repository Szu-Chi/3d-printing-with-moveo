
#ifndef ARCUS_EXPORT_H
#define ARCUS_EXPORT_H

#ifdef ARCUS_STATIC_DEFINE
#  define ARCUS_EXPORT
#  define ARCUS_NO_EXPORT
#else
#  ifndef ARCUS_EXPORT
#    ifdef MAKE_ARCUS_LIB
        /* We are building this library */
#      define ARCUS_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define ARCUS_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef ARCUS_NO_EXPORT
#    define ARCUS_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef ARCUS_DEPRECATED
#  define ARCUS_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef ARCUS_DEPRECATED_EXPORT
#  define ARCUS_DEPRECATED_EXPORT ARCUS_EXPORT ARCUS_DEPRECATED
#endif

#ifndef ARCUS_DEPRECATED_NO_EXPORT
#  define ARCUS_DEPRECATED_NO_EXPORT ARCUS_NO_EXPORT ARCUS_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef ARCUS_NO_DEPRECATED
#    define ARCUS_NO_DEPRECATED
#  endif
#endif

#endif
