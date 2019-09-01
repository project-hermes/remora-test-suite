/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9.2 at Tue Jul  2 09:07:54 2019. */

#include "dive.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t Dive_fields[12] = {
    PB_FIELD(  1, STRING  , SINGULAR, CALLBACK, FIRST, Dive, sensorId, sensorId, 0),
    PB_FIELD(  2, STRING  , SINGULAR, CALLBACK, OTHER, Dive, sensorType, sensorId, 0),
    PB_FIELD(  3, STRING  , SINGULAR, CALLBACK, OTHER, Dive, firmwareVersion, sensorType, 0),
    PB_FIELD(  4, STRING  , SINGULAR, CALLBACK, OTHER, Dive, schemaVersion, firmwareVersion, 0),
    PB_FIELD(  5, INT64   , SINGULAR, STATIC  , OTHER, Dive, startTime, schemaVersion, 0),
    PB_FIELD(  6, INT64   , SINGULAR, STATIC  , OTHER, Dive, endTime, startTime, 0),
    PB_FIELD(  7, FLOAT   , SINGULAR, STATIC  , OTHER, Dive, startLat, endTime, 0),
    PB_FIELD(  8, FLOAT   , SINGULAR, STATIC  , OTHER, Dive, startLong, startLat, 0),
    PB_FIELD(  9, FLOAT   , SINGULAR, STATIC  , OTHER, Dive, endLat, startLong, 0),
    PB_FIELD( 10, FLOAT   , SINGULAR, STATIC  , OTHER, Dive, endLong, endLat, 0),
    PB_REPEATED_FIXED_COUNT( 11, MESSAGE , OTHER, Dive, diveData, endLong, &DataPoint_fields),
    PB_LAST_FIELD
};

const pb_field_t DataPoint_fields[4] = {
    PB_FIELD(  1, FLOAT   , SINGULAR, STATIC  , FIRST, DataPoint, temp, temp, 0),
    PB_FIELD(  2, FLOAT   , SINGULAR, STATIC  , OTHER, DataPoint, depth, temp, 0),
    PB_FIELD(  3, FLOAT   , SINGULAR, STATIC  , OTHER, DataPoint, conductivity, depth, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(Dive, diveData[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_Dive_DataPoint)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for Dive.diveData is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* @@protoc_insertion_point(eof) */
