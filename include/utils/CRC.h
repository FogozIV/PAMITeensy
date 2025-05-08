//
// Created by fogoz on 02/05/2025.
//

#ifndef PAMITEENSY_CRC_H
#define PAMITEENSY_CRC_H
#include <cctype>
#include "vector"
#include <string>
/*
 * Source : WIKIPEDIA
 * https://en.wikipedia.org/wiki/Cyclic_redundancy_check
 * https://sourceforge.net/projects/crccalculator/
 */


template<typename crc>
class CRC_Algo{
protected:
    crc initialValue;
    crc final_xor_value;
    bool reversed_data;
    bool reversed_out;
    uint8_t width;
    crc polynomial;

    crc lookup_table[256];
    bool computed;
    std::function<crc(crc data)> reflect_data;
    std::function<crc(crc data)> reflect_crc_table;


    static crc FP_reflect(crc VF_data, crc nbit)
    {
        crc result = 0;

        for (crc index = 0; index < nbit; index++)
        {
            if ((VF_data & 1) == 1)
            {
                result |= (((crc)1) << ((nbit - 1) - index));
            }

            VF_data = (VF_data >> 1);
        }
        return (result);
    }

    crc getTableValue(uint8_t value)
    {
        crc result = 0;

        result = ((crc) reflect_data(value)) << (width - 8);

        for (uint8_t _ = 0; _ < 8; _++)
        {
            if (result & getTOPBit())
            {
                result = (result << 1) ^ polynomial;
            }
            else
            {
                result = (result << 1);
            }
        }
        return (reflect_crc_table(result));
    }
public:
    crc getTOPBit() const {
        return ((crc)1) << (width - 1);
    }

    void computeTable(){
        if(computed)
           return;
        computed = true;
        for (uint16_t index = 0; index < 256; index++)
        {
            lookup_table[index] = getTableValue((uint8_t)(index &0xFF));
        }
    }
    crc computeCRC(std::vector<uint8_t> input_data){
        return computeCRC(input_data.data(), input_data.size());
    }

    crc computeCRC(std::string str){
        return computeCRC(reinterpret_cast<const uint8_t *>(str.c_str()), str.size());
    }

    crc computeCRC(uint8_t const input_data[], uint16_t size_data)
    {
        crc	result = initialValue;
        computeTable();
        uint16_t index = 0;

        for (index = 0; index < size_data; index++)
        {
            if(reversed_data){
                result = (result >> 8) ^ lookup_table[((uint8_t)(result & 0xFF)) ^ input_data[index]];
            }else{
                result = (result << 8) ^ lookup_table[((uint8_t)((result >> (width - 8)) & 0xFF)) ^ input_data[index]];
            }
        }
        if((8 * sizeof(crc)) > width){
            uint64_t selector = (((crc)1) << width) - 1;
            result &= selector;
        }
        if(!reversed_out) {
            return (result ^ final_xor_value);
        }
        return (~result ^ final_xor_value);

    }
};
//ALGO(NAME, size, POLYNOMIAL, INITIAL_VALUE, FINAL_XOR_VALUE, REVERSED_DATA, REVERSED_OUT)
#define ALGORITHMS_CRC \
    ALGO(CRC_8, uint8_t, 0x07, 0, 0, false, false, 8) \
    ALGO(CRC_CCITT, uint16_t, 0x1021, 0xFFFF, 0, false, false, 16) \
    ALGO(MODBUS, uint16_t, 0x8005, 0xFFFF, 0, true, false, 16) \
    ALGO(CRC_16, uint16_t, 0x8005, 0x0000, 0x0000, true, false, 16) \
    ALGO(CRC_24, uint32_t, 0x864CFB, 0xB704CE, 0x000000, false, false, 24) \
    ALGO(CRC_32, uint32_t, 0x04C11DB7, 0x00000000, 0xFFFFFFFF, false, false, 32) \
    ALGO(CRC_32_BZIP2, uint32_t, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, false, false, 32) \
    ALGO(CRC_64_JONES, uint64_t, 0xAD93D23594C935A9, 0xFFFFFFFFFFFFFFFF, 0, true, false, 64)

#define ALGO(NAME, size, POLYNOMIAL, INITIAL_VALUE, FINAL_XOR_VALUE, REVERSED_DATA, REVERSED_OUT, WIDTH) \
    class NAME##_algo : public CRC_Algo<size>{                                                              \
        public:                                                                                             \
            NAME##_algo(){\
                initialValue = INITIAL_VALUE;                                                                \
                final_xor_value = FINAL_XOR_VALUE;                                                           \
                reversed_data = REVERSED_DATA;                                                               \
                reversed_out= REVERSED_OUT;                                                                  \
                width = WIDTH;                                                                               \
                polynomial = POLYNOMIAL;                                                                     \
                if(reversed_data){                                                                           \
                    reflect_data = [&](uint8_t data){return this->FP_reflect(data, 8)&0xFF;};                \
                }else{                                                                                       \
                    reflect_data = [&](uint8_t data){ return data;};                                         \
                }                                                                                        \
                if(reversed_out){                                                                        \
                    reflect_crc_table = [&](size data){return this->FP_reflect(data, width);};        \
                }else {                                                                                  \
                    reflect_crc_table = [&](size data){ return data;};                                \
                }\
                computeTable();\
            }\
    };\
    inline NAME##_algo algo##NAME;

ALGORITHMS_CRC
#undef ALGO
#undef ALGORITHMS_CRC

#endif //PAMITEENSY_CRC_H
