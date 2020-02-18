//
// Created by alpha on 2/18/20.
//

#ifndef PX4_RESERVOIR_COMPUTER_HPP
#define PX4_RESERVOIR_COMPUTER_HPP

#include <stdint.h>

class reservoir_computer {
public:
    explicit reservoir_computer(uint8_t reservoir_dimension, uint8_t output_dimension, input_dimension, float noise_d, float noise_g, float time_constant, float IS);

};


#endif //PX4_RESERVOIR_COMPUTER_HPP
