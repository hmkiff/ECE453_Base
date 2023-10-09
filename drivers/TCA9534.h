#ifndef TCA9534_H_
#define TCA9534_H_

cy_rslt_t TCA9534_send_command(uint8_t cmd_byte, uint8_t data_byte)
int TCA9534_get_input_port();
void TCA9534_set_configuration(int config);
void TCA9534_set_output_port(int port);

#endif