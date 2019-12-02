/*
 * Module code.
 *
 * Generated by SIP 4.19.19
 */
#line 134 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Types.sip"
#include "Types.h"
#line 9 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart2.cpp"

#include "sipAPIArcus.h"

#line 22 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/PythonMessage.sip"
    #include "PythonMessage.h"
#line 15 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart2.cpp"

#line 23 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Types.sip"
#include <string>
#line 19 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart2.cpp"


class sipPythonMessage : public  ::PythonMessage
{
public:
    sipPythonMessage(const  ::PythonMessage&);
    ~sipPythonMessage();

public:
    sipSimpleWrapper *sipPySelf;

private:
    sipPythonMessage(const sipPythonMessage &);
    sipPythonMessage &operator = (const sipPythonMessage &);
};

sipPythonMessage::sipPythonMessage(const  ::PythonMessage& a0):  ::PythonMessage(a0), sipPySelf(SIP_NULLPTR)
{
}

sipPythonMessage::~sipPythonMessage()
{
    sipInstanceDestroyedEx(&sipPySelf);
}


extern "C" {static PyObject *meth_PythonMessage_getTypeName(PyObject *, PyObject *);}
static PyObject *meth_PythonMessage_getTypeName(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::PythonMessage *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_PythonMessage, &sipCpp))
        {
             ::std::string*sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = new  ::std::string(sipCpp->getTypeName());
            Py_END_ALLOW_THREADS

            return sipConvertFromNewType(sipRes,sipType_std_string,SIP_NULLPTR);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, sipName_getTypeName, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_PythonMessage___hasattr__(PyObject *, PyObject *);}
static PyObject *meth_PythonMessage___hasattr__(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
        const  ::PythonMessage *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_PythonMessage, &sipCpp, sipType_std_string,&a0, &a0State))
        {
            bool sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->__hasattr__(*a0);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return PyBool_FromLong(sipRes);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, sipName___hasattr__, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_PythonMessage___getattr__(PyObject *, PyObject *);}
static PyObject *meth_PythonMessage___getattr__(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
        const  ::PythonMessage *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_PythonMessage, &sipCpp, sipType_std_string,&a0, &a0State))
        {
            PyObject * sipRes;

            sipRes = sipCpp->__getattr__(*a0);
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return sipRes;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, sipName___getattr__, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_PythonMessage_addRepeatedMessage(PyObject *, PyObject *);}
static PyObject *meth_PythonMessage_addRepeatedMessage(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
         ::PythonMessage *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_PythonMessage, &sipCpp, sipType_std_string,&a0, &a0State))
        {
             ::PythonMessage*sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->addRepeatedMessage(*a0);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return sipConvertFromType(sipRes,sipType_PythonMessage,Py_None);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, sipName_addRepeatedMessage, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_PythonMessage_repeatedMessageCount(PyObject *, PyObject *);}
static PyObject *meth_PythonMessage_repeatedMessageCount(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
        const  ::PythonMessage *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_PythonMessage, &sipCpp, sipType_std_string,&a0, &a0State))
        {
            int sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->repeatedMessageCount(*a0);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return SIPLong_FromLong(sipRes);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, sipName_repeatedMessageCount, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_PythonMessage_getRepeatedMessage(PyObject *, PyObject *);}
static PyObject *meth_PythonMessage_getRepeatedMessage(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
        int a1;
         ::PythonMessage *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1i", &sipSelf, sipType_PythonMessage, &sipCpp, sipType_std_string,&a0, &a0State, &a1))
        {
             ::PythonMessage*sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->getRepeatedMessage(*a0,a1);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return sipConvertFromType(sipRes,sipType_PythonMessage,Py_None);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, sipName_getRepeatedMessage, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_PythonMessage_getMessage(PyObject *, PyObject *);}
static PyObject *meth_PythonMessage_getMessage(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
         ::PythonMessage *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_PythonMessage, &sipCpp, sipType_std_string,&a0, &a0State))
        {
             ::PythonMessage*sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->getMessage(*a0);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return sipConvertFromType(sipRes,sipType_PythonMessage,Py_None);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, sipName_getMessage, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_PythonMessage_getEnumValue(PyObject *, PyObject *);}
static PyObject *meth_PythonMessage_getEnumValue(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
        const  ::PythonMessage *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ1", &sipSelf, sipType_PythonMessage, &sipCpp, sipType_std_string,&a0, &a0State))
        {
            int sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->getEnumValue(*a0);
            Py_END_ALLOW_THREADS
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return SIPLong_FromLong(sipRes);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, sipName_getEnumValue, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static int slot_PythonMessage___setattr__(PyObject *,PyObject *,PyObject *);}
static int slot_PythonMessage___setattr__(PyObject *sipSelf,PyObject *sipName,PyObject *sipValue)
{
     ::PythonMessage *sipCpp = reinterpret_cast< ::PythonMessage *>(sipGetCppPtr((sipSimpleWrapper *)sipSelf,sipType_PythonMessage));

    if (!sipCpp)
        return -1;

    PyObject *sipParseErr = SIP_NULLPTR;

    {
        const  ::std::string* a0;
        int a0State = 0;
        PyObject * a1;

        if (sipValue != SIP_NULLPTR && sipParsePair(&sipParseErr, sipName, sipValue, "J1P0", sipType_std_string,&a0, &a0State, &a1))
        {
#line 34 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/PythonMessage.sip"
    sipCpp->__setattr__(*a0, a1);
#line 301 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart2.cpp"
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return 0;
        }
    }

    {
        const  ::std::string* a0;
        int a0State = 0;

        if (sipValue == SIP_NULLPTR && sipParsePair(&sipParseErr, sipName, SIP_NULLPTR, "J1", sipType_std_string,&a0, &a0State))
        {
#line 39 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/PythonMessage.sip"
    PyErr_SetString(PyExc_NotImplementedError, "__delattr__ not supported on messages.");
    return 0;
#line 317 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart2.cpp"
            sipReleaseType(const_cast< ::std::string *>(a0),sipType_std_string,a0State);

            return 0;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_PythonMessage, (sipValue != SIP_NULLPTR ? sipName___setattr__ : sipName___delattr__), SIP_NULLPTR);

    return -1;
}


/* Call the instance's destructor. */
extern "C" {static void release_PythonMessage(void *, int);}
static void release_PythonMessage(void *sipCppV, int sipState)
{
    Py_BEGIN_ALLOW_THREADS

    if (sipState & SIP_DERIVED_CLASS)
        delete reinterpret_cast<sipPythonMessage *>(sipCppV);
    else
        delete reinterpret_cast< ::PythonMessage *>(sipCppV);

    Py_END_ALLOW_THREADS
}


extern "C" {static void dealloc_PythonMessage(sipSimpleWrapper *);}
static void dealloc_PythonMessage(sipSimpleWrapper *sipSelf)
{
    if (sipIsDerivedClass(sipSelf))
        reinterpret_cast<sipPythonMessage *>(sipGetAddress(sipSelf))->sipPySelf = SIP_NULLPTR;

    if (sipIsOwnedByPython(sipSelf))
    {
        release_PythonMessage(sipGetAddress(sipSelf), sipIsDerivedClass(sipSelf));
    }
}


extern "C" {static void *init_type_PythonMessage(sipSimpleWrapper *, PyObject *, PyObject *, PyObject **, PyObject **, PyObject **);}
static void *init_type_PythonMessage(sipSimpleWrapper *sipSelf, PyObject *sipArgs, PyObject *sipKwds, PyObject **sipUnused, PyObject **, PyObject **sipParseErr)
{
    sipPythonMessage *sipCpp = SIP_NULLPTR;

    {
        const  ::PythonMessage* a0;

        if (sipParseKwdArgs(sipParseErr, sipArgs, sipKwds, SIP_NULLPTR, sipUnused, "J9", sipType_PythonMessage, &a0))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp = new sipPythonMessage(*a0);
            Py_END_ALLOW_THREADS

            sipCpp->sipPySelf = sipSelf;

            return sipCpp;
        }
    }

    return SIP_NULLPTR;
}


/* Define this type's Python slots. */
static sipPySlotDef slots_PythonMessage[] = {
    {(void *)slot_PythonMessage___setattr__, setattr_slot},
    {0, (sipPySlotType)0}
};


static PyMethodDef methods_PythonMessage[] = {
    {SIP_MLNAME_CAST(sipName___getattr__), meth_PythonMessage___getattr__, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName___hasattr__), meth_PythonMessage___hasattr__, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_addRepeatedMessage), meth_PythonMessage_addRepeatedMessage, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_getEnumValue), meth_PythonMessage_getEnumValue, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_getMessage), meth_PythonMessage_getMessage, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_getRepeatedMessage), meth_PythonMessage_getRepeatedMessage, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_getTypeName), meth_PythonMessage_getTypeName, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_repeatedMessageCount), meth_PythonMessage_repeatedMessageCount, METH_VARARGS, SIP_NULLPTR}
};


sipClassTypeDef sipTypeDef_Arcus_PythonMessage = {
    {
        -1,
        SIP_NULLPTR,
        SIP_NULLPTR,
        SIP_TYPE_NONLAZY|SIP_TYPE_SUPER_INIT|SIP_TYPE_CLASS,
        sipNameNr_PythonMessage,
        {SIP_NULLPTR},
        SIP_NULLPTR
    },
    {
        sipNameNr_PythonMessage,
        {0, 0, 1},
        8, methods_PythonMessage,
        0, SIP_NULLPTR,
        0, SIP_NULLPTR,
        {SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR},
    },
    SIP_NULLPTR,
    -1,
    -1,
    SIP_NULLPTR,
    slots_PythonMessage,
    init_type_PythonMessage,
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
    dealloc_PythonMessage,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    release_PythonMessage,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR
};

#line 22 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/SocketListener.sip"
    #include "SocketListener.h"
#line 453 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart2.cpp"

#line 43 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Error.sip"
    #include "Error.h"
#line 457 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart2.cpp"
#line 33 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/python/Socket.sip"
    #include "Socket.h"
#line 460 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libArcus/build/python/sipArcuspart2.cpp"


class sipSocketListener : public  ::SocketListener
{
public:
    sipSocketListener();
    sipSocketListener(const  ::SocketListener&);
    virtual ~sipSocketListener();

    /*
     * There is a protected method for every virtual method visible from
     * this class.
     */
protected:
    void error(const  ::Error&) SIP_OVERRIDE;
    void messageReceived() SIP_OVERRIDE;
    void stateChanged( ::SocketState::SocketState) SIP_OVERRIDE;

public:
    sipSimpleWrapper *sipPySelf;

private:
    sipSocketListener(const sipSocketListener &);
    sipSocketListener &operator = (const sipSocketListener &);

    char sipPyMethods[3];
};

sipSocketListener::sipSocketListener():  ::SocketListener(), sipPySelf(SIP_NULLPTR)
{
    memset(sipPyMethods, 0, sizeof (sipPyMethods));
}

sipSocketListener::sipSocketListener(const  ::SocketListener& a0):  ::SocketListener(a0), sipPySelf(SIP_NULLPTR)
{
    memset(sipPyMethods, 0, sizeof (sipPyMethods));
}

sipSocketListener::~sipSocketListener()
{
    sipInstanceDestroyedEx(&sipPySelf);
}

void sipSocketListener::error(const  ::Error& a0)
{
    sip_gilstate_t sipGILState;
    PyObject *sipMeth;

    sipMeth = sipIsPyMethod(&sipGILState,&sipPyMethods[0],sipPySelf,sipName_SocketListener,sipName_error);

    if (!sipMeth)
        return;

    extern void sipVH_Arcus_2(sip_gilstate_t, sipVirtErrorHandlerFunc, sipSimpleWrapper *, PyObject *, const  ::Error&);

    sipVH_Arcus_2(sipGILState, 0, sipPySelf, sipMeth, a0);
}

void sipSocketListener::messageReceived()
{
    sip_gilstate_t sipGILState;
    PyObject *sipMeth;

    sipMeth = sipIsPyMethod(&sipGILState,&sipPyMethods[1],sipPySelf,sipName_SocketListener,sipName_messageReceived);

    if (!sipMeth)
        return;

    extern void sipVH_Arcus_1(sip_gilstate_t, sipVirtErrorHandlerFunc, sipSimpleWrapper *, PyObject *);

    sipVH_Arcus_1(sipGILState, 0, sipPySelf, sipMeth);
}

void sipSocketListener::stateChanged( ::SocketState::SocketState a0)
{
    sip_gilstate_t sipGILState;
    PyObject *sipMeth;

    sipMeth = sipIsPyMethod(&sipGILState,&sipPyMethods[2],sipPySelf,sipName_SocketListener,sipName_stateChanged);

    if (!sipMeth)
        return;

    extern void sipVH_Arcus_0(sip_gilstate_t, sipVirtErrorHandlerFunc, sipSimpleWrapper *, PyObject *,  ::SocketState::SocketState);

    sipVH_Arcus_0(sipGILState, 0, sipPySelf, sipMeth, a0);
}


extern "C" {static PyObject *meth_SocketListener_getSocket(PyObject *, PyObject *);}
static PyObject *meth_SocketListener_getSocket(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;

    {
         ::SocketListener *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_SocketListener, &sipCpp))
        {
             ::Socket*sipRes;

            Py_BEGIN_ALLOW_THREADS
            sipRes = sipCpp->getSocket();
            Py_END_ALLOW_THREADS

            return sipConvertFromType(sipRes,sipType_Socket,SIP_NULLPTR);
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_SocketListener, sipName_getSocket, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_SocketListener_stateChanged(PyObject *, PyObject *);}
static PyObject *meth_SocketListener_stateChanged(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;
    PyObject *sipOrigSelf = sipSelf;

    {
         ::SocketState::SocketState a0;
         ::SocketListener *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BE", &sipSelf, sipType_SocketListener, &sipCpp, sipType_SocketState_SocketState, &a0))
        {
            if (!sipOrigSelf)
            {
                sipAbstractMethod(sipName_SocketListener, sipName_stateChanged);
                return SIP_NULLPTR;
            }

            sipCpp->stateChanged(a0);

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_SocketListener, sipName_stateChanged, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_SocketListener_messageReceived(PyObject *, PyObject *);}
static PyObject *meth_SocketListener_messageReceived(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;
    PyObject *sipOrigSelf = sipSelf;

    {
         ::SocketListener *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "B", &sipSelf, sipType_SocketListener, &sipCpp))
        {
            if (!sipOrigSelf)
            {
                sipAbstractMethod(sipName_SocketListener, sipName_messageReceived);
                return SIP_NULLPTR;
            }

            sipCpp->messageReceived();

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_SocketListener, sipName_messageReceived, SIP_NULLPTR);

    return SIP_NULLPTR;
}


extern "C" {static PyObject *meth_SocketListener_error(PyObject *, PyObject *);}
static PyObject *meth_SocketListener_error(PyObject *sipSelf, PyObject *sipArgs)
{
    PyObject *sipParseErr = SIP_NULLPTR;
    PyObject *sipOrigSelf = sipSelf;

    {
        const  ::Error* a0;
         ::SocketListener *sipCpp;

        if (sipParseArgs(&sipParseErr, sipArgs, "BJ9", &sipSelf, sipType_SocketListener, &sipCpp, sipType_Error, &a0))
        {
            if (!sipOrigSelf)
            {
                sipAbstractMethod(sipName_SocketListener, sipName_error);
                return SIP_NULLPTR;
            }

            sipCpp->error(*a0);

            Py_INCREF(Py_None);
            return Py_None;
        }
    }

    /* Raise an exception if the arguments couldn't be parsed. */
    sipNoMethod(sipParseErr, sipName_SocketListener, sipName_error, SIP_NULLPTR);

    return SIP_NULLPTR;
}


/* Call the instance's destructor. */
extern "C" {static void release_SocketListener(void *, int);}
static void release_SocketListener(void *sipCppV, int sipState)
{
    Py_BEGIN_ALLOW_THREADS

    if (sipState & SIP_DERIVED_CLASS)
        delete reinterpret_cast<sipSocketListener *>(sipCppV);
    else
        delete reinterpret_cast< ::SocketListener *>(sipCppV);

    Py_END_ALLOW_THREADS
}


extern "C" {static void dealloc_SocketListener(sipSimpleWrapper *);}
static void dealloc_SocketListener(sipSimpleWrapper *sipSelf)
{
    if (sipIsDerivedClass(sipSelf))
        reinterpret_cast<sipSocketListener *>(sipGetAddress(sipSelf))->sipPySelf = SIP_NULLPTR;

    if (sipIsOwnedByPython(sipSelf))
    {
        release_SocketListener(sipGetAddress(sipSelf), sipIsDerivedClass(sipSelf));
    }
}


extern "C" {static void *init_type_SocketListener(sipSimpleWrapper *, PyObject *, PyObject *, PyObject **, PyObject **, PyObject **);}
static void *init_type_SocketListener(sipSimpleWrapper *sipSelf, PyObject *sipArgs, PyObject *sipKwds, PyObject **sipUnused, PyObject **, PyObject **sipParseErr)
{
    sipSocketListener *sipCpp = SIP_NULLPTR;

    {
        if (sipParseKwdArgs(sipParseErr, sipArgs, sipKwds, SIP_NULLPTR, sipUnused, ""))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp = new sipSocketListener();
            Py_END_ALLOW_THREADS

            sipCpp->sipPySelf = sipSelf;

            return sipCpp;
        }
    }

    {
        const  ::SocketListener* a0;

        if (sipParseKwdArgs(sipParseErr, sipArgs, sipKwds, SIP_NULLPTR, sipUnused, "J9", sipType_SocketListener, &a0))
        {
            Py_BEGIN_ALLOW_THREADS
            sipCpp = new sipSocketListener(*a0);
            Py_END_ALLOW_THREADS

            sipCpp->sipPySelf = sipSelf;

            return sipCpp;
        }
    }

    return SIP_NULLPTR;
}


static PyMethodDef methods_SocketListener[] = {
    {SIP_MLNAME_CAST(sipName_error), meth_SocketListener_error, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_getSocket), meth_SocketListener_getSocket, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_messageReceived), meth_SocketListener_messageReceived, METH_VARARGS, SIP_NULLPTR},
    {SIP_MLNAME_CAST(sipName_stateChanged), meth_SocketListener_stateChanged, METH_VARARGS, SIP_NULLPTR}
};


sipClassTypeDef sipTypeDef_Arcus_SocketListener = {
    {
        -1,
        SIP_NULLPTR,
        SIP_NULLPTR,
        SIP_TYPE_ABSTRACT|SIP_TYPE_SUPER_INIT|SIP_TYPE_CLASS,
        sipNameNr_SocketListener,
        {SIP_NULLPTR},
        SIP_NULLPTR
    },
    {
        sipNameNr_SocketListener,
        {0, 0, 1},
        4, methods_SocketListener,
        0, SIP_NULLPTR,
        0, SIP_NULLPTR,
        {SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR, SIP_NULLPTR},
    },
    SIP_NULLPTR,
    -1,
    -1,
    SIP_NULLPTR,
    SIP_NULLPTR,
    init_type_SocketListener,
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
    dealloc_SocketListener,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    release_SocketListener,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR,
    SIP_NULLPTR
};