#include "NEC.h"

typedef struct
{
	uint8_t adr_l;
	uint8_t adr_h;
	uint8_t cmd;
	uint8_t ncmd;
}nec_msg_mask_t __attribute__((aligned (8)));

#define NEC_MIN_T_SQ (3000)//минимальная длинна стартового импульса
#define NEC_MAX_T_SQ (6000)

#define NEC_MIN_T_SQ_R (1500)//минимальная длинна стартового импульса
#define NEC_MAX_T_SQ_R (4000)


void NEC_decode_callback(nec_ir_t* Nec)
{
	uint32_t i=0;
	uint32_t cnt=0;
	uint32_t temp=0;
	nec_msg_mask_t* p=(nec_msg_mask_t*)&temp;
	
	if (Nec->_cnt==4)//проверка повтора
	{
		if (Nec->_len_pulse_raw[2]>=10000&&
				Nec->_len_pulse_raw[2]-Nec->_len_pulse_raw[3]>=NEC_MIN_T_SQ_R&&
				Nec->_len_pulse_raw[2]-Nec->_len_pulse_raw[3]<=NEC_MAX_T_SQ_R)
		{
			Nec->_cnt_repeat++;
			if (Nec->_cnt_repeat==2)
			{
				Nec->_cnt_repeat=0;
				Nec->NEC_RepeatCallback();
			}
		}
	}
	else
	{
		Nec->_cnt_repeat=0;
		for (i=0;i<Nec->_cnt;i++)//поиск стартовой последовательности
		{
			if (Nec->_len_pulse_raw[i]>=Nec->start_seq_time&&
					Nec->_len_pulse_raw[i]-Nec->_len_pulse_raw[i+1]>=NEC_MIN_T_SQ&&
					Nec->_len_pulse_raw[i]-Nec->_len_pulse_raw[i+1]<=NEC_MAX_T_SQ)
			{
				i+=2;//перехожу к данным
				break;
			}
			else if (i>16)//ошибка в пакете
			{
				//Nec->addr=0;
				//Nec->cmd=0;
				return;
			}
		}
	
		for (;i<Nec->_cnt;i+=2)//обрабатываю массив
		{
			if ((Nec->_len_pulse_raw[i]-Nec->_len_pulse_raw[i+1])>=Nec->bit_time) temp|=1<<cnt;
			if (cnt<32) cnt++;
			else 
			{
				break;
			}
		}
	
		if (Nec->type==EXTENDED_TYPE)
		{
			p->ncmd=~p->ncmd;
		
			if (p->cmd==p->ncmd)
			{	
				Nec->cmd=p->cmd;
				Nec->addr=((p->adr_h)<<8)|(p->adr_l);
				Nec->NEC_DecodedCallback(Nec->addr, Nec->cmd);
			}
			else;//ошибка команды
		}
		else//стандартный протокол
		{
			p->adr_h=~p->adr_h;
			p->ncmd =~p->ncmd;
			if ((p->adr_l==p->adr_h)&&(p->cmd==p->ncmd))
			{
				Nec->addr=p->adr_l;
				Nec->cmd=p->cmd;
			}
			else ;//ошибка
		}
	}
}
