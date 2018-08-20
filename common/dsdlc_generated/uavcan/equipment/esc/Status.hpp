/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/isthatme/Documents/SPEAR/embedded/common/libuavcan/dsdl/uavcan/equipment/esc/1034.Status.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_ESC_STATUS_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_ESC_STATUS_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Generic ESC status.
# Unknown fields should be set to NAN.
#

uint32 error_count          # Resets when the motor restarts

float16 voltage             # Volt
float16 current             # Ampere. Can be negative in case of a regenerative braking.
float16 temperature         # Kelvin

int18 rpm                   # Negative value indicates reverse rotation

uint7 power_rating_pct      # Instant demand factor in percent (percent of maximum power); range 0% to 127%.

uint5 esc_index
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.esc.Status
saturated uint32 error_count
saturated float16 voltage
saturated float16 current
saturated float16 temperature
saturated int18 rpm
saturated uint7 power_rating_pct
saturated uint5 esc_index
******************************************************************************/

#undef error_count
#undef voltage
#undef current
#undef temperature
#undef rpm
#undef power_rating_pct
#undef esc_index

namespace uavcan
{
namespace equipment
{
namespace esc
{

template <int _tmpl>
struct UAVCAN_EXPORT Status_
{
    typedef const Status_<_tmpl>& ParameterType;
    typedef Status_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > error_count;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > voltage;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > current;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > temperature;
        typedef ::uavcan::IntegerSpec< 18, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > rpm;
        typedef ::uavcan::IntegerSpec< 7, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > power_rating_pct;
        typedef ::uavcan::IntegerSpec< 5, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > esc_index;
    };

    enum
    {
        MinBitLen
            = FieldTypes::error_count::MinBitLen
            + FieldTypes::voltage::MinBitLen
            + FieldTypes::current::MinBitLen
            + FieldTypes::temperature::MinBitLen
            + FieldTypes::rpm::MinBitLen
            + FieldTypes::power_rating_pct::MinBitLen
            + FieldTypes::esc_index::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::error_count::MaxBitLen
            + FieldTypes::voltage::MaxBitLen
            + FieldTypes::current::MaxBitLen
            + FieldTypes::temperature::MaxBitLen
            + FieldTypes::rpm::MaxBitLen
            + FieldTypes::power_rating_pct::MaxBitLen
            + FieldTypes::esc_index::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::error_count >::Type error_count;
    typename ::uavcan::StorageType< typename FieldTypes::voltage >::Type voltage;
    typename ::uavcan::StorageType< typename FieldTypes::current >::Type current;
    typename ::uavcan::StorageType< typename FieldTypes::temperature >::Type temperature;
    typename ::uavcan::StorageType< typename FieldTypes::rpm >::Type rpm;
    typename ::uavcan::StorageType< typename FieldTypes::power_rating_pct >::Type power_rating_pct;
    typename ::uavcan::StorageType< typename FieldTypes::esc_index >::Type esc_index;

    Status_()
        : error_count()
        , voltage()
        , current()
        , temperature()
        , rpm()
        , power_rating_pct()
        , esc_index()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<110 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1034 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.esc.Status";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool Status_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        error_count == rhs.error_count &&
        voltage == rhs.voltage &&
        current == rhs.current &&
        temperature == rhs.temperature &&
        rpm == rhs.rpm &&
        power_rating_pct == rhs.power_rating_pct &&
        esc_index == rhs.esc_index;
}

template <int _tmpl>
bool Status_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(error_count, rhs.error_count) &&
        ::uavcan::areClose(voltage, rhs.voltage) &&
        ::uavcan::areClose(current, rhs.current) &&
        ::uavcan::areClose(temperature, rhs.temperature) &&
        ::uavcan::areClose(rpm, rhs.rpm) &&
        ::uavcan::areClose(power_rating_pct, rhs.power_rating_pct) &&
        ::uavcan::areClose(esc_index, rhs.esc_index);
}

template <int _tmpl>
int Status_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::error_count::encode(self.error_count, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::voltage::encode(self.voltage, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::current::encode(self.current, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::temperature::encode(self.temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rpm::encode(self.rpm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::power_rating_pct::encode(self.power_rating_pct, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::esc_index::encode(self.esc_index, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Status_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::error_count::decode(self.error_count, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::voltage::decode(self.voltage, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::current::decode(self.current, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::temperature::decode(self.temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rpm::decode(self.rpm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::power_rating_pct::decode(self.power_rating_pct, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::esc_index::decode(self.esc_index, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Status_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xA9AF28AEA2FBB254ULL);

    FieldTypes::error_count::extendDataTypeSignature(signature);
    FieldTypes::voltage::extendDataTypeSignature(signature);
    FieldTypes::current::extendDataTypeSignature(signature);
    FieldTypes::temperature::extendDataTypeSignature(signature);
    FieldTypes::rpm::extendDataTypeSignature(signature);
    FieldTypes::power_rating_pct::extendDataTypeSignature(signature);
    FieldTypes::esc_index::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Status_<0> Status;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::esc::Status > _uavcan_gdtr_registrator_Status;

}

} // Namespace esc
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::esc::Status >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::esc::Status::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::esc::Status >::stream(Stream& s, ::uavcan::equipment::esc::Status::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "error_count: ";
    YamlStreamer< ::uavcan::equipment::esc::Status::FieldTypes::error_count >::stream(s, obj.error_count, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "voltage: ";
    YamlStreamer< ::uavcan::equipment::esc::Status::FieldTypes::voltage >::stream(s, obj.voltage, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "current: ";
    YamlStreamer< ::uavcan::equipment::esc::Status::FieldTypes::current >::stream(s, obj.current, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "temperature: ";
    YamlStreamer< ::uavcan::equipment::esc::Status::FieldTypes::temperature >::stream(s, obj.temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "rpm: ";
    YamlStreamer< ::uavcan::equipment::esc::Status::FieldTypes::rpm >::stream(s, obj.rpm, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "power_rating_pct: ";
    YamlStreamer< ::uavcan::equipment::esc::Status::FieldTypes::power_rating_pct >::stream(s, obj.power_rating_pct, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "esc_index: ";
    YamlStreamer< ::uavcan::equipment::esc::Status::FieldTypes::esc_index >::stream(s, obj.esc_index, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace esc
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::esc::Status::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::esc::Status >::stream(s, obj, 0);
    return s;
}

} // Namespace esc
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_ESC_STATUS_HPP_INCLUDED