/*
 * Module code.
 *
 * Generated by SIP 4.19.19
 */
#line 134 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Types.sip"
#include "Types.h"
#line 9 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart0.cpp"

#include "sipAPIArcus.h"

#line 43 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Error.sip"
    #include "Error.h"
#line 15 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart0.cpp"

/* Define the strings used by this module. */
const char sipStrings_Arcus[] = {
    'M', 'e', 's', 's', 'a', 'g', 'e', 'R', 'e', 'g', 'i', 's', 't', 'r', 'a', 't', 'i', 'o', 'n', 'F', 'a', 'i', 'l', 'e', 'd', 'E', 'r', 'r', 'o', 'r', 0,
    'S', 'o', 'c', 'k', 'e', 't', 'S', 't', 'a', 't', 'e', ':', ':', 'S', 'o', 'c', 'k', 'e', 't', 'S', 't', 'a', 't', 'e', 0,
    'r', 'e', 'g', 'i', 's', 't', 'e', 'r', 'A', 'l', 'l', 'M', 'e', 's', 's', 'a', 'g', 'e', 'T', 'y', 'p', 'e', 's', 0,
    'U', 'n', 'k', 'n', 'o', 'w', 'n', 'M', 'e', 's', 's', 'a', 'g', 'e', 'T', 'y', 'p', 'e', 'E', 'r', 'r', 'o', 'r', 0,
    'C', 'o', 'n', 'n', 'e', 'c', 't', 'i', 'o', 'n', 'R', 'e', 's', 'e', 't', 'E', 'r', 'r', 'o', 'r', 0,
    'E', 'r', 'r', 'o', 'r', 'C', 'o', 'd', 'e', ':', ':', 'E', 'r', 'r', 'o', 'r', 'C', 'o', 'd', 'e', 0,
    'r', 'e', 'p', 'e', 'a', 't', 'e', 'd', 'M', 'e', 's', 's', 'a', 'g', 'e', 'C', 'o', 'u', 'n', 't', 0,
    'I', 'n', 'v', 'a', 'l', 'i', 'd', 'M', 'e', 's', 's', 'a', 'g', 'e', 'E', 'r', 'r', 'o', 'r', 0,
    'R', 'e', 'c', 'e', 'i', 'v', 'e', 'F', 'a', 'i', 'l', 'e', 'd', 'E', 'r', 'r', 'o', 'r', 0,
    'C', 'o', 'n', 'n', 'e', 'c', 't', 'F', 'a', 'i', 'l', 'e', 'd', 'E', 'r', 'r', 'o', 'r', 0,
    'g', 'e', 't', 'R', 'e', 'p', 'e', 'a', 't', 'e', 'd', 'M', 'e', 's', 's', 'a', 'g', 'e', 0,
    'a', 'd', 'd', 'R', 'e', 'p', 'e', 'a', 't', 'e', 'd', 'M', 'e', 's', 's', 'a', 'g', 'e', 0,
    'I', 'n', 'v', 'a', 'l', 'i', 'd', 'S', 't', 'a', 't', 'e', 'E', 'r', 'r', 'o', 'r', 0,
    'A', 'c', 'c', 'e', 'p', 't', 'F', 'a', 'i', 'l', 'e', 'd', 'E', 'r', 'r', 'o', 'r', 0,
    'P', 'a', 'r', 's', 'e', 'F', 'a', 'i', 'l', 'e', 'd', 'E', 'r', 'r', 'o', 'r', 0,
    't', 'a', 'k', 'e', 'N', 'e', 'x', 't', 'M', 'e', 's', 's', 'a', 'g', 'e', 0,
    'g', 'e', 't', 'E', 'r', 'r', 'o', 'r', 'M', 'e', 's', 's', 'a', 'g', 'e', 0,
    'S', 'e', 'n', 'd', 'F', 'a', 'i', 'l', 'e', 'd', 'E', 'r', 'r', 'o', 'r', 0,
    'B', 'i', 'n', 'd', 'F', 'a', 'i', 'l', 'e', 'd', 'E', 'r', 'r', 'o', 'r', 0,
    'm', 'e', 's', 's', 'a', 'g', 'e', 'R', 'e', 'c', 'e', 'i', 'v', 'e', 'd', 0,
    'r', 'e', 'm', 'o', 'v', 'e', 'L', 'i', 's', 't', 'e', 'n', 'e', 'r', 0,
    'S', 'o', 'c', 'k', 'e', 't', 'L', 'i', 's', 't', 'e', 'n', 'e', 'r', 0,
    'c', 'r', 'e', 'a', 't', 'e', 'M', 'e', 's', 's', 'a', 'g', 'e', 0,
    's', 'e', 't', 'F', 'a', 't', 'a', 'l', 'E', 'r', 'r', 'o', 'r', 0,
    'C', 'r', 'e', 'a', 't', 'i', 'o', 'n', 'E', 'r', 'r', 'o', 'r', 0,
    'P', 'y', 't', 'h', 'o', 'n', 'M', 'e', 's', 's', 'a', 'g', 'e', 0,
    'g', 'e', 't', 'L', 'a', 's', 't', 'E', 'r', 'r', 'o', 'r', 0,
    'i', 's', 'F', 'a', 't', 'a', 'l', 'E', 'r', 'r', 'o', 'r', 0,
    'g', 'e', 't', 'E', 'r', 'r', 'o', 'r', 'C', 'o', 'd', 'e', 0,
    'U', 'n', 'k', 'n', 'o', 'w', 'n', 'E', 'r', 'r', 'o', 'r', 0,
    'g', 'e', 't', 'E', 'n', 'u', 'm', 'V', 'a', 'l', 'u', 'e', 0,
    's', 't', 'a', 't', 'e', 'C', 'h', 'a', 'n', 'g', 'e', 'd', 0,
    's', 'e', 'n', 'd', 'M', 'e', 's', 's', 'a', 'g', 'e', 0,
    'a', 'd', 'd', 'L', 'i', 's', 't', 'e', 'n', 'e', 'r', 0,
    '_', '_', 'd', 'e', 'l', 'a', 't', 't', 'r', '_', '_', 0,
    '_', '_', 's', 'e', 't', 'a', 't', 't', 'r', '_', '_', 0,
    '_', '_', 'g', 'e', 't', 'a', 't', 't', 'r', '_', '_', 0,
    '_', '_', 'h', 'a', 's', 'a', 't', 't', 'r', '_', '_', 0,
    'g', 'e', 't', 'T', 'y', 'p', 'e', 'N', 'a', 'm', 'e', 0,
    's', 't', 'd', ':', ':', 's', 't', 'r', 'i', 'n', 'g', 0,
    'c', 'l', 'e', 'a', 'r', 'E', 'r', 'r', 'o', 'r', 0,
    'g', 'e', 't', 'M', 'e', 's', 's', 'a', 'g', 'e', 0,
    'C', 'o', 'n', 'n', 'e', 'c', 't', 'i', 'n', 'g', 0,
    'M', 'e', 's', 's', 'a', 'g', 'e', 'P', 't', 'r', 0,
    'g', 'e', 't', 'S', 'o', 'c', 'k', 'e', 't', 0,
    'L', 'i', 's', 't', 'e', 'n', 'i', 'n', 'g', 0,
    'C', 'o', 'n', 'n', 'e', 'c', 't', 'e', 'd', 0,
    'g', 'e', 't', 'S', 't', 'a', 't', 'e', 0,
    '_', '_', 'r', 'e', 'p', 'r', '_', '_', 0,
    'c', 'o', 'n', 'n', 'e', 'c', 't', 0,
    'i', 's', 'V', 'a', 'l', 'i', 'd', 0,
    'C', 'l', 'o', 's', 'i', 'n', 'g', 0,
    'O', 'p', 'e', 'n', 'i', 'n', 'g', 0,
    'I', 'n', 'i', 't', 'i', 'a', 'l', 0,
    'l', 'i', 's', 't', 'e', 'n', 0,
    'C', 'l', 'o', 's', 'e', 'd', 0,
    'r', 'e', 's', 'e', 't', 0,
    'c', 'l', 'o', 's', 'e', 0,
    'D', 'e', 'b', 'u', 'g', 0,
    'e', 'r', 'r', 'o', 'r', 0,
    'A', 'r', 'c', 'u', 's', 0,
};

void sipVH_Arcus_2(sip_gilstate_t sipGILState, sipVirtErrorHandlerFunc sipErrorHandler, sipSimpleWrapper *sipPySelf, PyObject *sipMethod, const  ::Error& a0)
{
    sipCallProcedureMethod(sipGILState, sipErrorHandler, sipPySelf, sipMethod, "N", new  ::Error(a0), sipType_Error, SIP_NULLPTR);
}

void sipVH_Arcus_1(sip_gilstate_t sipGILState, sipVirtErrorHandlerFunc sipErrorHandler, sipSimpleWrapper *sipPySelf, PyObject *sipMethod)
{
    sipCallProcedureMethod(sipGILState, sipErrorHandler, sipPySelf, sipMethod, "");
}

void sipVH_Arcus_0(sip_gilstate_t sipGILState, sipVirtErrorHandlerFunc sipErrorHandler, sipSimpleWrapper *sipPySelf, PyObject *sipMethod,  ::SocketState::SocketState a0)
{
    sipCallProcedureMethod(sipGILState, sipErrorHandler, sipPySelf, sipMethod, "F", a0, sipType_SocketState_SocketState);
}
static sipEnumTypeDef enumTypes[] = {
    {{-1, 0, 0, SIP_TYPE_ENUM, sipNameNr_ErrorCode__ErrorCode, {0}, 0}, sipNameNr_ErrorCode, 1, SIP_NULLPTR},
    {{-1, 0, 0, SIP_TYPE_ENUM, sipNameNr_SocketState__SocketState, {0}, 0}, sipNameNr_SocketState, 7, SIP_NULLPTR},
};


/*
 * This defines each type in this module.
 */
sipTypeDef *sipExportedTypes_Arcus[] = {
    &sipTypeDef_Arcus_Error.ctd_base,
    &sipTypeDef_Arcus_ErrorCode.ctd_base,
    &enumTypes[0].etd_base,
    &sipTypeDef_Arcus_MessagePtr.mtd_base,
    &sipTypeDef_Arcus_PythonMessage.ctd_base,
    &sipTypeDef_Arcus_Socket.ctd_base,
    &sipTypeDef_Arcus_SocketListener.ctd_base,
    &sipTypeDef_Arcus_SocketState.ctd_base,
    &enumTypes[1].etd_base,
    &sipTypeDef_Arcus_std_string.mtd_base,
};


/* This defines this module. */
sipExportedModuleDef sipModuleAPI_Arcus = {
    0,
    SIP_API_MINOR_NR,
    sipNameNr_Arcus,
    0,
    sipStrings_Arcus,
    SIP_NULLPTR,
    SIP_NULLPTR,
    10,
    sipExportedTypes_Arcus,
    SIP_NULLPTR,
    0,
    SIP_NULLPTR,
    0,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    {SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR},
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR
};


/* The SIP API and the APIs of any imported modules. */
const sipAPIDef *sipAPI_Arcus;


/* The Python module initialisation function. */
#if PY_MAJOR_VERSION >= 3
#define SIP_MODULE_ENTRY        PyInit_Arcus
#define SIP_MODULE_TYPE         PyObject *
#define SIP_MODULE_DISCARD(r)   Py_DECREF(r)
#define SIP_MODULE_RETURN(r)    return (r)
#else
#define SIP_MODULE_ENTRY        initArcus
#define SIP_MODULE_TYPE         void
#define SIP_MODULE_DISCARD(r)
#define SIP_MODULE_RETURN(r)    return
#endif

#if defined(SIP_STATIC_MODULE)
extern "C" SIP_MODULE_TYPE SIP_MODULE_ENTRY()
#else
PyMODINIT_FUNC SIP_MODULE_ENTRY()
#endif
{
    static PyMethodDef sip_methods[] = {
        {SIP_NULLPTR, SIP_NULLPTR, 0, SIP_NULLPTR}
    };

#if PY_MAJOR_VERSION >= 3
    static PyModuleDef sip_module_def = {
        PyModuleDef_HEAD_INIT,
        "Arcus",
        SIP_NULLPTR,
        -1,
        sip_methods,
        SIP_NULLPTR,
        SIP_NULLPTR,
        SIP_NULLPTR,
        SIP_NULLPTR
    };
#endif

    PyObject *sipModule, *sipModuleDict;
    PyObject *sip_sipmod, *sip_capiobj;

    /* Initialise the module and get it's dictionary. */
#if PY_MAJOR_VERSION >= 3
    sipModule = PyModule_Create(&sip_module_def);
#elif PY_VERSION_HEX >= 0x02050000
    sipModule = Py_InitModule(sipName_Arcus, sip_methods);
#else
    sipModule = Py_InitModule(const_cast<char *>(sipName_Arcus), sip_methods);
#endif

    if (sipModule == SIP_NULLPTR)
        SIP_MODULE_RETURN(SIP_NULLPTR);

    sipModuleDict = PyModule_GetDict(sipModule);

    /* Get the SIP module's API. */
#if PY_VERSION_HEX >= 0x02050000
    sip_sipmod = PyImport_ImportModule("sip");
#else
    sip_sipmod = PyImport_ImportModule(const_cast<char *>("sip"));
#endif

    if (sip_sipmod == SIP_NULLPTR)
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(SIP_NULLPTR);
    }

    sip_capiobj = PyDict_GetItemString(PyModule_GetDict(sip_sipmod), "_C_API");
    Py_DECREF(sip_sipmod);

#if defined(SIP_USE_PYCAPSULE)
    if (sip_capiobj == SIP_NULLPTR || !PyCapsule_CheckExact(sip_capiobj))
#else
    if (sip_capiobj == SIP_NULLPTR || !PyCObject_Check(sip_capiobj))
#endif
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(SIP_NULLPTR);
    }

#if defined(SIP_USE_PYCAPSULE)
    sipAPI_Arcus = reinterpret_cast<const sipAPIDef *>(PyCapsule_GetPointer(sip_capiobj, "sip._C_API"));
#else
    sipAPI_Arcus = reinterpret_cast<const sipAPIDef *>(PyCObject_AsVoidPtr(sip_capiobj));
#endif

#if defined(SIP_USE_PYCAPSULE)
    if (sipAPI_Arcus == SIP_NULLPTR)
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(SIP_NULLPTR);
    }
#endif

    /* Export the module and publish it's API. */
    if (sipExportModule(&sipModuleAPI_Arcus,SIP_API_MAJOR_NR,SIP_API_MINOR_NR,0) < 0)
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(SIP_NULLPTR);
    }
    /* Initialise the module now all its dependencies have been set up. */
    if (sipInitModule(&sipModuleAPI_Arcus,sipModuleDict) < 0)
    {
        SIP_MODULE_DISCARD(sipModule);
        SIP_MODULE_RETURN(SIP_NULLPTR);
    }

    SIP_MODULE_RETURN(sipModule);
}

#line 33 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Socket.sip"
    #include "Socket.h"
#line 265 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart0.cpp"

#line 23 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Types.sip"
#include <string>
#line 269 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart0.cpp"
#line 72 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Types.sip"
#include <memory>
#include "PythonMessage.h"
#line 273 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart0.cpp"
#line 22 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/SocketListener.sip"
    #include "SocketListener.h"
#line 276 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart0.cpp"
#line 43 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Error.sip"
    #include "Error.h"
#line 279 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart0.cpp"


class sipSocket : public  ::Socket
{
public:
    sipSocket();
    ~sipSocket();

public:
    sipSimpleWrapper *sipPySelf;

private:
    sipSocket(const sipSocket &);
    sipSocket &operator = (const sipSocket &);
};

sipSocket::sipSocket():  ::Socket(), sipPySelf(SIP_NULLPTR)
{
}

sipSocket::~sipSocket()
{
    sipInstanceDestroyedEx(&sipPySelf);
}


extern "C" {static PyObject *meth_Socket_getState(PyObject *, PyObject *);}
static PyObject *meth_Socket_getState(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_Socket, &sipCpp))
        {
             ::SocketState::SocketState sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->getState();
            Py_END_ALLOW_THREADS

            return sipConvertFromEnum(static_cast<int>(sipRes), sipType_SocketState_SocketState);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_getState, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_getLastError(PyObject *, PyObject *);}
static PyObject *meth_Socket_getLastError(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_Socket, &sipCpp))
        {
             ::Error*sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = new  ::Error(sipCpp->getLastError());
            Py_END_ALLOW_THREADS

            return sipConvertFromNewType(sipRes,sipType_Error,SIP_NULLPTR);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_getLastError, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_clearError(PyObject *, PyObject *);}
static PyObject *meth_Socket_clearError(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_Socket, &sipCpp))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp->clearError();
            Py_END_ALLOW_THREADS

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_clearError, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_addListener(PyObject *, PyObject *);}
static PyObject *meth_Socket_addListener(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
         ::SocketListener* a0;
        sipWrapper *sipOwner = SIP_NULLPTR;
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJH", &sipSelf, sipType_Socket, &sipCpp, sipType_SocketListener, &a0, &sipOwner))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp->addListener(a0);
            Py_END_ALLOW_THREADS

            if (sipOwner)
                sipTransferTo(sipSelf, (PyObject *)sipOwner);
            else
                sipTransferBack(sipSelf);

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_addListener, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_removeListener(PyObject *, PyObject *);}
static PyObject *meth_Socket_removeListener(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
         ::SocketListener* a0;
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ8", &sipSelf, sipType_Socket, &sipCpp, sipType_SocketListener, &a0))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp->removeListener(a0);
            Py_END_ALLOW_THREADS

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_removeListener, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_connect(PyObject *, PyObject *);}
static PyObject *meth_Socket_connect(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
        int a1;
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1i", &sipSelf, sipType_Socket, &sipCpp, sipType_std_string,&a0, &a0State, &a1))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp->connect(*a0,a1);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_connect, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_listen(PyObject *, PyObject *);}
static PyObject *meth_Socket_listen(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
        int a1;
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1i", &sipSelf, sipType_Socket, &sipCpp, sipType_std_string,&a0, &a0State, &a1))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp->listen(*a0,a1);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_listen, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_close(PyObject *, PyObject *);}
static PyObject *meth_Socket_close(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_Socket, &sipCpp))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp->close();
            Py_END_ALLOW_THREADS

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_close, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_reset(PyObject *, PyObject *);}
static PyObject *meth_Socket_reset(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_Socket, &sipCpp))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp->reset();
            Py_END_ALLOW_THREADS

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_reset, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_sendMessage(PyObject *, PyObject *);}
static PyObject *meth_Socket_sendMessage(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
         ::MessagePtr* a0;
        int a0State = 0;
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_Socket, &sipCpp, sipType_MessagePtr,&a0, &a0State))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp->sendMessage(*a0);
            Py_END_ALLOW_THREADS
            sipReleaseType(a0,sipType_MessagePtr,a0State);

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_sendMessage, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_takeNextMessage(PyObject *, PyObject *);}
static PyObject *meth_Socket_takeNextMessage(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_Socket, &sipCpp))
        {
             ::MessagePtr*sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = new  ::MessagePtr(sipCpp->takeNextMessage());
            Py_END_ALLOW_THREADS

            return sipConvertFromNewType(sipRes,sipType_MessagePtr,SIP_NULLPTR);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_takeNextMessage, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_createMessage(PyObject *, PyObject *);}
static PyObject *meth_Socket_createMessage(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_Socket, &sipCpp, sipType_std_string,&a0, &a0State))
        {
             ::MessagePtr*sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = new  ::MessagePtr(sipCpp->createMessage(*a0));
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return sipConvertFromNewType(sipRes,sipType_MessagePtr,SIP_NULLPTR);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_createMessage, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_Socket_registerAllMessageTypes(PyObject *, PyObject *);}
static PyObject *meth_Socket_registerAllMessageTypes(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
         ::Socket *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_Socket, &sipCpp, sipType_std_string,&a0, &a0State))
        {
            bool sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->registerAllMessageTypes(*a0);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return PyBool_FromLong(sipRes);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_Socket, sipName_registerAllMessageTypes, SIP_NULLPTR);

    return SIP_NULLPTR;
}


/* Call the instance's destructor. */
extern "C" {static void release_Socket(void *, int);}
static void release_Socket(void *sipCppV, int sipState)
{
    Py_BEGIN_ALLOW_THREADS

    if (sipState & SIP_DERIVED_CLASS)
        delete reinterpret_cast<sipSocket *>(sipCppV);
    else
        delete reinterpret_cast< ::Socket *>(sipCppV);

    Py_END_ALLOW_THREADS
}


extern "C" {static void dealloc_Socket(sipSimpleWrapper *);}
static void dealloc_Socket(sipSimpleWrapper *sipSelf)
{
    if (sipIsDerivedClass(sipSelf))
        reinterpret_cast<sipSocket *>(sipGetAddress(sipSelf))->sipPySelf = SIP_NULLPTR;

    if (sipIsOwnedByPython(sipSelf))
    {
        release_Socket(sipGetAddress(sipSelf), sipIsDerivedClass(sipSelf));
    }
}


extern "C" {static void *init_type_Socket(sipSimpleWrapper *, PyObject *, PyObject *, PyObject **, PyObject **, PyObject **);}
static void *init_type_Socket(sipSimpleWrapper *sipSelf, PyObject *sipArgs, PyObject *sipKwds, PyObject **sipUnused, PyObject **, PyObject **sipParseErr)
{
    sipSocket *sipCpp = SIP_NULLPTR;

    {
        if (sipParseKwdArgs(sipParseErr, sipArgs, sipKwds, SIP_NULLPTR, sipUnused, ""))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp = new sipSocket();
            Py_END_ALLOW_THREADS

            sipCpp->sipPySelf = sipSelf;

            return sipCpp;
        }
    }

    return SIP_NULLPTR;
}


static PyMethodDef methods_Socket[] = {
    {SIP_MLNAME_CAST(sipName_addListener), meth_Socket_addListener, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_clearError), meth_Socket_clearError, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_close), meth_Socket_close, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_connect), meth_Socket_connect, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_createMessage), meth_Socket_createMessage, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_getLastError), meth_Socket_getLastError, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_getState), meth_Socket_getState, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_listen), meth_Socket_listen, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_registerAllMessageTypes), meth_Socket_registerAllMessageTypes, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_removeListener), meth_Socket_removeListener, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_reset), meth_Socket_reset, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_sendMessage), meth_Socket_sendMessage, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_takeNextMessage), meth_Socket_takeNextMessage, METH_VARARGS, SIP_NULLPTR}
};


sipClassTypeDef sipTypeDef_Arcus_Socket = {
    {
        -1,
        SIP_NULLPTR,
        SIP_NULLPTR,
        SIP_TYPE_SUPER_INIT|SIP_TYPE_CLASS,
        sipNameNr_Socket,
        {SIP_NULLPTR},
        SIP_NULLPTR
    },
    {
        sipNameNr_Socket,
        {0, 0, 1},
        13, methods_Socket,
        0, SIP_NULLPTR,
        0, SIP_NULLPTR,
        {SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR},
    },
    SIP_NULLPTR,
    -1,
    -1,
    SIP_NULLPTR,
    SIP_NULLPTR,
    init_type_Socket,
    SIP_NULLPTR,
    SIP_NULLPTR,
#if PY_MAJOR_VERSION >= 3
    SIP_NULLPTR,
    SIP_NULLPTR,
#else
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
#endif
    dealloc_Socket,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    release_Socket,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR
};
