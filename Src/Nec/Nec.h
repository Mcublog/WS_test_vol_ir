#ifndef NEC_H
#define NEC_H

#include <stdint.h>

#define BUFF_LEN (255)
typedef enum
{
	EXTENDED_TYPE,
	STANDART_TYPE
}nec_type_t;

typedef struct 
{
	nec_type_t type;
	
	uint16_t addr;//адрес 
	uint8_t  cmd;//команда
	
	uint32_t start_seq_time;//длительность стартовой последовательности
	uint32_t bit_time;//длительность одного байта
	
	uint32_t _cnt;//кол-во принятых бит
	uint16_t _len_pulse_raw[BUFF_LEN];//буффер сырых данных
	
	uint32_t _cnt_repeat;//кол-во повторов
	
  void (*NEC_DecodedCallback)(uint16_t, uint8_t);
  void (*NEC_ErrorCallback)();
	void (*NEC_RepeatCallback)();	
}nec_ir_t;

void NEC_decode_callback(nec_ir_t* Nec);

#endif
