
#ifndef SAVITAR_EXPORT_H
#define SAVITAR_EXPORT_H

#ifdef SAVITAR_STATIC_DEFINE
#  define SAVITAR_EXPORT
#  define SAVITAR_NO_EXPORT
#else
#  ifndef SAVITAR_EXPORT
#    ifdef MAKE_SAVITAR_LIB
        /* We are building this library */
#      define SAVITAR_EXPORT 
#    else
        /* We are using this library */
#      define SAVITAR_EXPORT 
#    endif
#  endif

#  ifndef SAVITAR_NO_EXPORT
#    define SAVITAR_NO_EXPORT 
#  endif
#endif

#ifndef SAVITAR_DEPRECATED
#  define SAVITAR_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef SAVITAR_DEPRECATED_EXPORT
#  define SAVITAR_DEPRECATED_EXPORT SAVITAR_EXPORT SAVITAR_DEPRECATED
#endif

#ifndef SAVITAR_DEPRECATED_NO_EXPORT
#  define SAVITAR_DEPRECATED_NO_EXPORT SAVITAR_NO_EXPORT SAVITAR_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef SAVITAR_NO_DEPRECATED
#    define SAVITAR_NO_DEPRECATED
#  endif
#endif

#endif
