/*
 * Internal module API header file.
 *
 * Generated by SIP 4.19.19
 */

#ifndef _SavitarAPI_H
#define _SavitarAPI_H

#include <sip.h>

/*
 * Convenient names to refer to various strings defined in this module.
 * Only the class names are part of the public API.
 */
#define sipNameNr_0 0
#define sipName_0 &sipStrings_Savitar[0]
#define sipNameNr_34 34
#define sipName_34 &sipStrings_Savitar[34]
#define sipNameNr_getFlatVerticesAsBytes 58
#define sipName_getFlatVerticesAsBytes &sipStrings_Savitar[58]
#define sipNameNr_setVerticesFromBytes 81
#define sipName_setVerticesFromBytes &sipStrings_Savitar[81]
#define sipNameNr_getVerticesAsBytes 102
#define sipName_getVerticesAsBytes &sipStrings_Savitar[102]
#define sipNameNr_setFacesFromBytes 121
#define sipName_setFacesFromBytes &sipStrings_Savitar[121]
#define sipNameNr_setTransformation 139
#define sipName_setTransformation &sipStrings_Savitar[139]
#define sipNameNr_getTransformation 157
#define sipName_getTransformation &sipStrings_Savitar[157]
#define sipNameNr_getAllSceneNodes 175
#define sipName_getAllSceneNodes &sipStrings_Savitar[175]
#define sipNameNr_getFacesAsBytes 192
#define sipName_getFacesAsBytes &sipStrings_Savitar[192]
#define sipNameNr_sceneToString 208
#define sipName_sceneToString &sipStrings_Savitar[208]
#define sipNameNr_ThreeMFParser 222
#define sipName_ThreeMFParser &sipStrings_Savitar[222]
#define sipNameNr_getSceneNodes 236
#define sipName_getSceneNodes &sipStrings_Savitar[236]
#define sipNameNr_addSceneNode 250
#define sipName_addSceneNode &sipStrings_Savitar[250]
#define sipNameNr_getMetadata 263
#define sipName_getMetadata &sipStrings_Savitar[263]
#define sipNameNr_getSettings 275
#define sipName_getSettings &sipStrings_Savitar[275]
#define sipNameNr_getChildren 287
#define sipName_getChildren &sipStrings_Savitar[287]
#define sipNameNr_getMeshData 299
#define sipName_getMeshData &sipStrings_Savitar[299]
#define sipNameNr_std__string 311
#define sipName_std__string &sipStrings_Savitar[311]
#define sipNameNr_setSetting 323
#define sipName_setSetting &sipStrings_Savitar[323]
#define sipNameNr_SceneNode 253
#define sipName_SceneNode &sipStrings_Savitar[253]
#define sipNameNr_bytearray 334
#define sipName_bytearray &sipStrings_Savitar[334]
#define sipNameNr_MeshData 302
#define sipName_MeshData &sipStrings_Savitar[302]
#define sipNameNr_addChild 344
#define sipName_addChild &sipStrings_Savitar[344]
#define sipNameNr_setUnit 353
#define sipName_setUnit &sipStrings_Savitar[353]
#define sipNameNr_getUnit 361
#define sipName_getUnit &sipStrings_Savitar[361]
#define sipNameNr_Savitar 369
#define sipName_Savitar &sipStrings_Savitar[369]
#define sipNameNr_string 316
#define sipName_string &sipStrings_Savitar[316]
#define sipNameNr_parse 377
#define sipName_parse &sipStrings_Savitar[377]
#define sipNameNr_Scene 383
#define sipName_Scene &sipStrings_Savitar[383]

#define sipMalloc                   sipAPI_Savitar->api_malloc
#define sipFree                     sipAPI_Savitar->api_free
#define sipBuildResult              sipAPI_Savitar->api_build_result
#define sipCallMethod               sipAPI_Savitar->api_call_method
#define sipCallProcedureMethod      sipAPI_Savitar->api_call_procedure_method
#define sipCallErrorHandler         sipAPI_Savitar->api_call_error_handler
#define sipParseResultEx            sipAPI_Savitar->api_parse_result_ex
#define sipParseResult              sipAPI_Savitar->api_parse_result
#define sipParseArgs                sipAPI_Savitar->api_parse_args
#define sipParseKwdArgs             sipAPI_Savitar->api_parse_kwd_args
#define sipParsePair                sipAPI_Savitar->api_parse_pair
#define sipInstanceDestroyed        sipAPI_Savitar->api_instance_destroyed
#define sipInstanceDestroyedEx      sipAPI_Savitar->api_instance_destroyed_ex
#define sipConvertFromSequenceIndex sipAPI_Savitar->api_convert_from_sequence_index
#define sipConvertFromSliceObject   sipAPI_Savitar->api_convert_from_slice_object
#define sipConvertFromVoidPtr       sipAPI_Savitar->api_convert_from_void_ptr
#define sipConvertToVoidPtr         sipAPI_Savitar->api_convert_to_void_ptr
#define sipAddException             sipAPI_Savitar->api_add_exception
#define sipNoFunction               sipAPI_Savitar->api_no_function
#define sipNoMethod                 sipAPI_Savitar->api_no_method
#define sipAbstractMethod           sipAPI_Savitar->api_abstract_method
#define sipBadClass                 sipAPI_Savitar->api_bad_class
#define sipBadCatcherResult         sipAPI_Savitar->api_bad_catcher_result
#define sipBadCallableArg           sipAPI_Savitar->api_bad_callable_arg
#define sipBadOperatorArg           sipAPI_Savitar->api_bad_operator_arg
#define sipTrace                    sipAPI_Savitar->api_trace
#define sipTransferBack             sipAPI_Savitar->api_transfer_back
#define sipTransferTo               sipAPI_Savitar->api_transfer_to
#define sipTransferBreak            sipAPI_Savitar->api_transfer_break
#define sipSimpleWrapper_Type       sipAPI_Savitar->api_simplewrapper_type
#define sipWrapper_Type             sipAPI_Savitar->api_wrapper_type
#define sipWrapperType_Type         sipAPI_Savitar->api_wrappertype_type
#define sipVoidPtr_Type             sipAPI_Savitar->api_voidptr_type
#define sipGetPyObject              sipAPI_Savitar->api_get_pyobject
#define sipGetAddress               sipAPI_Savitar->api_get_address
#define sipGetMixinAddress          sipAPI_Savitar->api_get_mixin_address
#define sipGetCppPtr                sipAPI_Savitar->api_get_cpp_ptr
#define sipGetComplexCppPtr         sipAPI_Savitar->api_get_complex_cpp_ptr
#define sipIsPyMethod               sipAPI_Savitar->api_is_py_method
#define sipCallHook                 sipAPI_Savitar->api_call_hook
#define sipEndThread                sipAPI_Savitar->api_end_thread
#define sipConnectRx                sipAPI_Savitar->api_connect_rx
#define sipDisconnectRx             sipAPI_Savitar->api_disconnect_rx
#define sipRaiseUnknownException    sipAPI_Savitar->api_raise_unknown_exception
#define sipRaiseTypeException       sipAPI_Savitar->api_raise_type_exception
#define sipBadLengthForSlice        sipAPI_Savitar->api_bad_length_for_slice
#define sipAddTypeInstance          sipAPI_Savitar->api_add_type_instance
#define sipFreeSipslot              sipAPI_Savitar->api_free_sipslot
#define sipSameSlot                 sipAPI_Savitar->api_same_slot
#define sipPySlotExtend             sipAPI_Savitar->api_pyslot_extend
#define sipConvertRx                sipAPI_Savitar->api_convert_rx
#define sipAddDelayedDtor           sipAPI_Savitar->api_add_delayed_dtor
#define sipCanConvertToType         sipAPI_Savitar->api_can_convert_to_type
#define sipConvertToType            sipAPI_Savitar->api_convert_to_type
#define sipForceConvertToType       sipAPI_Savitar->api_force_convert_to_type
#define sipCanConvertToEnum         sipAPI_Savitar->api_can_convert_to_enum
#define sipConvertToEnum            sipAPI_Savitar->api_convert_to_enum
#define sipConvertToBool            sipAPI_Savitar->api_convert_to_bool
#define sipReleaseType              sipAPI_Savitar->api_release_type
#define sipConvertFromType          sipAPI_Savitar->api_convert_from_type
#define sipConvertFromNewType       sipAPI_Savitar->api_convert_from_new_type
#define sipConvertFromNewPyType     sipAPI_Savitar->api_convert_from_new_pytype
#define sipConvertFromEnum          sipAPI_Savitar->api_convert_from_enum
#define sipGetState                 sipAPI_Savitar->api_get_state
#define sipExportSymbol             sipAPI_Savitar->api_export_symbol
#define sipImportSymbol             sipAPI_Savitar->api_import_symbol
#define sipFindType                 sipAPI_Savitar->api_find_type
#define sipFindNamedEnum            sipAPI_Savitar->api_find_named_enum
#define sipBytes_AsChar             sipAPI_Savitar->api_bytes_as_char
#define sipBytes_AsString           sipAPI_Savitar->api_bytes_as_string
#define sipString_AsASCIIChar       sipAPI_Savitar->api_string_as_ascii_char
#define sipString_AsASCIIString     sipAPI_Savitar->api_string_as_ascii_string
#define sipString_AsLatin1Char      sipAPI_Savitar->api_string_as_latin1_char
#define sipString_AsLatin1String    sipAPI_Savitar->api_string_as_latin1_string
#define sipString_AsUTF8Char        sipAPI_Savitar->api_string_as_utf8_char
#define sipString_AsUTF8String      sipAPI_Savitar->api_string_as_utf8_string
#define sipUnicode_AsWChar          sipAPI_Savitar->api_unicode_as_wchar
#define sipUnicode_AsWString        sipAPI_Savitar->api_unicode_as_wstring
#define sipConvertFromConstVoidPtr  sipAPI_Savitar->api_convert_from_const_void_ptr
#define sipConvertFromVoidPtrAndSize    sipAPI_Savitar->api_convert_from_void_ptr_and_size
#define sipConvertFromConstVoidPtrAndSize   sipAPI_Savitar->api_convert_from_const_void_ptr_and_size
#define sipInvokeSlot               sipAPI_Savitar->api_invoke_slot
#define sipInvokeSlotEx             sipAPI_Savitar->api_invoke_slot_ex
#define sipSaveSlot                 sipAPI_Savitar->api_save_slot
#define sipClearAnySlotReference    sipAPI_Savitar->api_clear_any_slot_reference
#define sipVisitSlot                sipAPI_Savitar->api_visit_slot
#define sipWrappedTypeName(wt)      ((wt)->wt_td->td_cname)
#define sipDeprecated               sipAPI_Savitar->api_deprecated
#define sipGetReference             sipAPI_Savitar->api_get_reference
#define sipKeepReference            sipAPI_Savitar->api_keep_reference
#define sipRegisterProxyResolver    sipAPI_Savitar->api_register_proxy_resolver
#define sipRegisterPyType           sipAPI_Savitar->api_register_py_type
#define sipTypeFromPyTypeObject     sipAPI_Savitar->api_type_from_py_type_object
#define sipTypeScope                sipAPI_Savitar->api_type_scope
#define sipResolveTypedef           sipAPI_Savitar->api_resolve_typedef
#define sipRegisterAttributeGetter  sipAPI_Savitar->api_register_attribute_getter
#define sipIsAPIEnabled             sipAPI_Savitar->api_is_api_enabled
#define sipSetDestroyOnExit         sipAPI_Savitar->api_set_destroy_on_exit
#define sipEnableAutoconversion     sipAPI_Savitar->api_enable_autoconversion
#define sipEnableOverflowChecking   sipAPI_Savitar->api_enable_overflow_checking
#define sipInitMixin                sipAPI_Savitar->api_init_mixin
#define sipExportModule             sipAPI_Savitar->api_export_module
#define sipInitModule               sipAPI_Savitar->api_init_module
#define sipGetInterpreter           sipAPI_Savitar->api_get_interpreter
#define sipSetNewUserTypeHandler    sipAPI_Savitar->api_set_new_user_type_handler
#define sipSetTypeUserData          sipAPI_Savitar->api_set_type_user_data
#define sipGetTypeUserData          sipAPI_Savitar->api_get_type_user_data
#define sipPyTypeDict               sipAPI_Savitar->api_py_type_dict
#define sipPyTypeName               sipAPI_Savitar->api_py_type_name
#define sipGetCFunction             sipAPI_Savitar->api_get_c_function
#define sipGetMethod                sipAPI_Savitar->api_get_method
#define sipFromMethod               sipAPI_Savitar->api_from_method
#define sipGetDate                  sipAPI_Savitar->api_get_date
#define sipFromDate                 sipAPI_Savitar->api_from_date
#define sipGetDateTime              sipAPI_Savitar->api_get_datetime
#define sipFromDateTime             sipAPI_Savitar->api_from_datetime
#define sipGetTime                  sipAPI_Savitar->api_get_time
#define sipFromTime                 sipAPI_Savitar->api_from_time
#define sipIsUserType               sipAPI_Savitar->api_is_user_type
#define sipGetFrame                 sipAPI_Savitar->api_get_frame
#define sipCheckPluginForType       sipAPI_Savitar->api_check_plugin_for_type
#define sipUnicodeNew               sipAPI_Savitar->api_unicode_new
#define sipUnicodeWrite             sipAPI_Savitar->api_unicode_write
#define sipUnicodeData              sipAPI_Savitar->api_unicode_data
#define sipGetBufferInfo            sipAPI_Savitar->api_get_buffer_info
#define sipReleaseBufferInfo        sipAPI_Savitar->api_release_buffer_info
#define sipIsOwnedByPython          sipAPI_Savitar->api_is_owned_by_python
#define sipIsDerivedClass           sipAPI_Savitar->api_is_derived_class
#define sipGetUserObject            sipAPI_Savitar->api_get_user_object
#define sipSetUserObject            sipAPI_Savitar->api_set_user_object
#define sipRegisterEventHandler     sipAPI_Savitar->api_register_event_handler
#define sipLong_AsChar              sipAPI_Savitar->api_long_as_char
#define sipLong_AsSignedChar        sipAPI_Savitar->api_long_as_signed_char
#define sipLong_AsUnsignedChar      sipAPI_Savitar->api_long_as_unsigned_char
#define sipLong_AsShort             sipAPI_Savitar->api_long_as_short
#define sipLong_AsUnsignedShort     sipAPI_Savitar->api_long_as_unsigned_short
#define sipLong_AsInt               sipAPI_Savitar->api_long_as_int
#define sipLong_AsUnsignedInt       sipAPI_Savitar->api_long_as_unsigned_int
#define sipLong_AsLong              sipAPI_Savitar->api_long_as_long
#define sipLong_AsUnsignedLong      sipAPI_Savitar->api_long_as_unsigned_long
#define sipLong_AsLongLong          sipAPI_Savitar->api_long_as_long_long
#define sipLong_AsUnsignedLongLong  sipAPI_Savitar->api_long_as_unsigned_long_long
#define sipLong_AsSizeT             sipAPI_Savitar->api_long_as_size_t
#define sipVisitWrappers            sipAPI_Savitar->api_visit_wrappers
#define sipRegisterExitNotifier     sipAPI_Savitar->api_register_exit_notifier

/* These are deprecated. */
#define sipMapStringToClass         sipAPI_Savitar->api_map_string_to_class
#define sipMapIntToClass            sipAPI_Savitar->api_map_int_to_class
#define sipFindClass                sipAPI_Savitar->api_find_class
#define sipFindMappedType           sipAPI_Savitar->api_find_mapped_type
#define sipConvertToArray           sipAPI_Savitar->api_convert_to_array
#define sipConvertToTypedArray      sipAPI_Savitar->api_convert_to_typed_array
#define sipEnableGC                 sipAPI_Savitar->api_enable_gc
#define sipPrintObject              sipAPI_Savitar->api_print_object
#define sipWrapper_Check(w)         PyObject_TypeCheck((w), sipAPI_Savitar->api_wrapper_type)
#define sipGetWrapper(p, wt)        sipGetPyObject((p), (wt)->wt_td)
#define sipReleaseInstance(p, wt, s)    sipReleaseType((p), (wt)->wt_td, (s))
#define sipReleaseMappedType        sipReleaseType
#define sipCanConvertToInstance(o, wt, f)   sipCanConvertToType((o), (wt)->wt_td, (f))
#define sipCanConvertToMappedType   sipCanConvertToType
#define sipConvertToInstance(o, wt, t, f, s, e)     sipConvertToType((o), (wt)->wt_td, (t), (f), (s), (e))
#define sipConvertToMappedType      sipConvertToType
#define sipForceConvertToInstance(o, wt, t, f, s, e)    sipForceConvertToType((o), (wt)->wt_td, (t), (f), (s), (e))
#define sipForceConvertToMappedType sipForceConvertToType
#define sipConvertFromInstance(p, wt, t)    sipConvertFromType((p), (wt)->wt_td, (t))
#define sipConvertFromMappedType    sipConvertFromType
#define sipConvertFromNamedEnum(v, pt)  sipConvertFromEnum((v), ((sipEnumTypeObject *)(pt))->type)
#define sipConvertFromNewInstance(p, wt, t) sipConvertFromNewType((p), (wt)->wt_td, (t))

/* The strings used by this module. */
extern const char sipStrings_Savitar[];

#define sipType_SceneNode sipExportedTypes_Savitar[2]
#define sipClass_SceneNode sipExportedTypes_Savitar[2]->u.td_wrapper_type

extern sipClassTypeDef sipTypeDef_Savitar_SceneNode;

#define sipType_Scene sipExportedTypes_Savitar[1]
#define sipClass_Scene sipExportedTypes_Savitar[1]->u.td_wrapper_type

extern sipClassTypeDef sipTypeDef_Savitar_Scene;

#define sipType_MeshData sipExportedTypes_Savitar[0]
#define sipClass_MeshData sipExportedTypes_Savitar[0]->u.td_wrapper_type

extern sipClassTypeDef sipTypeDef_Savitar_MeshData;

#define sipType_ThreeMFParser sipExportedTypes_Savitar[3]
#define sipClass_ThreeMFParser sipExportedTypes_Savitar[3]->u.td_wrapper_type

extern sipClassTypeDef sipTypeDef_Savitar_ThreeMFParser;

#define sipType_std_map_0100std_string_0100std_string sipExportedTypes_Savitar[5]

extern sipMappedTypeDef sipTypeDef_Savitar_std_map_0100std_string_0100std_string;

#define sipType_bytearray sipExportedTypes_Savitar[4]

extern sipMappedTypeDef sipTypeDef_Savitar_bytearray;

#define sipType_std_vector_0101SceneNode sipExportedTypes_Savitar[7]

extern sipMappedTypeDef sipTypeDef_Savitar_std_vector_0101SceneNode;

#define sipType_std_string sipExportedTypes_Savitar[6]

extern sipMappedTypeDef sipTypeDef_Savitar_std_string;

/* The SIP API, this module's API and the APIs of any imported modules. */
extern const sipAPIDef *sipAPI_Savitar;
extern sipExportedModuleDef sipModuleAPI_Savitar;
extern sipTypeDef *sipExportedTypes_Savitar[];
#line 27 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/python/ThreeMFParser.sip"
namespace Savitar{} //Ensure that the namespace already exists
using namespace Savitar;
#line 294 "/home/tiao/git_ws/moveo_moveit_ws/src/Cura/libSavitar/build/python/sipAPISavitar.h"

#endif