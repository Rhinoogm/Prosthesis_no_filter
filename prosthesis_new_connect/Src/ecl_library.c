#include "ecl_library.h"
#define PI 3.141592653589793

extern UART_HandleTypeDef huart2;

ToHost toSimulink;

void S2D_calc_param(S2D *v)
{

	v->K = 2/(v->T);

	v->X0 = (v->b0*v->K*v->K + v->b1*v->K + v->b2)		/(v->a0*v->K*v->K + v->a1*v->K + v->a2);
	v->X1 = (2*v->b2 - 2*v->b0*v->K*v->K)				/(v->a0*v->K*v->K + v->a1*v->K + v->a2);
	v->X2 = (v->b0 * v->K*v->K - v->b1*v->K + v->b2)	/(v->a0*v->K*v->K + v->a1*v->K + v->a2);
	v->Y1 = (2*v->a2 - 2*v->a0*v->K*v->K)				/(v->a0*v->K*v->K + v->a1*v->K + v->a2);
	v->Y2 = (v->a0*v->K*v->K - v->a1*v->K + v->a2)		/(v->a0*v->K*v->K + v->a1*v->K + v->a2);
}

void S2D_calc(S2D *v)
{
	v->x = v->input;

	v->y =    v->X0	* (v->x)		\
			+ v->X1	* (v->x_p) 		\
			+ v->X2 * (v->x_pp)   	\
			- v->Y1	* (v->y_p) 		\
			- v->Y2	* (v->y_pp);

	v->x_pp = v->x_p;
	v->x_p	= v->x;

	v->y_pp = v->y_p;
	v->y_p	= v->y;
}


//--------------------------------------------------------------------
// ���⼭�� PID ����� �˰����� Simulink�� PID block�� ���� ���� �˰����� �״��
// �Űܳ��� ���̴�. Simulink PID block�� look under mask�� �ϰ� �Ǹ� ���θ� �� ��
// �ִ�. �Ʒ��� code�� ���� ����. �Ǵ� Prof. Lee�� �������� PPTX ���ǳ�Ʈ��
// DC ���� ����κ��� �����ϸ� block diagram�� ã�ƺ� �� �ִ�.
//--------------------------------------------------------------------
void pid_con_calc(PID_CON *v)
{
	v->Err = v->Ref - v->Fdb; // Compute the error

	v->Out_tmp = v->Kp*v->Err + v->Ki*v->Intg + v->Kd*v->Deriv; // PID controller output computation

	if(v->Out_tmp > v->OutMax){  // saturation check
		v->Out = v->OutMax;
	}
	else if(v->Out_tmp < v->OutMin) {
		v->Out = v->OutMin;
	}
	else{
		v->Out = v->Out_tmp;
	}

	v->SatErr = v->Out_tmp - v->Out;  // difference between PID output and saturated PID output
	v->Intg += v->Ts*(v->Err - v->Ka*v->SatErr);  // Integral value update
	v->Deriv = (v->Err - v->Err_past)/v->Ts;      // Derivative value update

	v->Err_past = v->Err;

}


void LPF_calc_param(LPF *v)
{
	//++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Parameter calulation follows the following rules.
	// 1�� LPF�� ���� Bilinear transform�� �̻��� filter��
	// ����Ѵ�.
	// �����Լ� : G(z) = k1(z+1)/(z+k2)
	// [���¹�����]
	// x(k+1) = -k2*x(k) + k_1*(1-k2)*u(k)
	//   y(k) = x(k) + k1*u(k)
	//++++++++++++++++++++++++++++++++++++++++++++++++++++
	// wc = 2*pi*fc
	// k1 = wc*Ts/(wc*Ts+2)
	// k2 = (wc*Ts-2)/(wc*Ts+2)
	//++++++++++++++++++++++++++++++++++++++++++++++++++++

	real_T Wc = 2.0*PI*v->Fc;
	v->k1 = Wc*v->Ts/(Wc*v->Ts +2.0);
	v->k2 = (Wc*v->Ts - 2.0)/(Wc*v->Ts + 2.0);
}

//-----------------------------------------------
// 1�� LPF�� ���¹����� 
//----------------------------------------------
// x(k+1) = -k2*x(k) + k_1*(1-k2)*u(k)
//   y(k) = x(k) + k1*u(k)
//-----------------------------------------------
void LPF_calc(LPF *v)
{
	v->Out = v->State + v->k1*v->In;  // Output calculation
	v->State = -v->k2*v->State + (v->k1)*(1.0-(v->k2))*v->In; // State update
}


void ENCODER_calc_param(ENCODER *v)
{
	v->h_rng = (s32)1<<(v->n_bits-1); // full range parameter
	v->f_rng = (s32)1<<(v->n_bits);   // half range parameter
	v->scale = (1.0/(4.0*(real_T)v->PPR))*2.0*PI; // scale factor computation

}

void ENCODER_calc(ENCODER *v)
{
	v->diff = v->Nc - v->Np; // Difference between the current and past counter value
	if(v->diff > v->h_rng)
	{
		v->diff = v->diff - v->f_rng;
	}
	else if(v->diff<-v->h_rng)
	{
		v->diff = v->diff + v->f_rng;
	}
	v->Na += v->diff;   // Accumulated counter value
	v->Np = v->Nc;      // Past counter value

	v->dot_theta_past = v->dot_theta;
	v->dot_theta = (real_T)v->diff*v->scale/v->Ts;  // Velocity computation

	v->dotdot_theta = (v->dot_theta - v->dot_theta_past)/v->Ts;

//	v->dotdot_theta = (real_T)v->diff*v->scale
	v->theta = (real_T)v->Na*v->scale;    // Current angle computation
}



//-----------------------------------------------------------
// PC�� ������ data�� ToHost structure�� �������ش�.
// data pointer �����ϴ� �Ͱ� data�� ��, channel number �����
// ������ �ش�.
//------------------------------------------------------------
void connectData(ToHost *toHostVar, char type, char channel, void *ptr)
{
	toHostVar->NumOfToHostBlock++;  // ToHostBlock�� ���� �ϳ��� ����
	if(type==SS_DOUBLE) // ���� double�� data �̸�
	{
		toHostVar->NumOfDoubleData++;  // Double�� data�� ���� 1�� ����
	}
	toHostVar->TypeAndChannel[channel] = (type<<4)|(channel &0x0F);
	toHostVar->DataPtr[channel] = ptr;  // pointer ����
	toHostVar->ModNumber = 510/(10*(toHostVar->NumOfToHostBlock-toHostVar->NumOfDoubleData) + 20*toHostVar->NumOfDoubleData + 2);
}


void putcToBuffer(ToHost *toHostVar, char c)
{
	toHostVar->data[toHostVar->indx++] = c;
	return;
}


void outhex8_Buffer(ToHost *toHostVar, char n)
{
	char c;
	c = (n >> 4) & 0x0000000f;
	if (c > 9)  toHostVar->data[toHostVar->indx++] = c - 10 + 'a';
	else toHostVar->data[toHostVar->indx++] = c+'0';

	c = n & 0x0000000f;
	if (c > 9) toHostVar->data[toHostVar->indx++] = c - 10 + 'a';
	else toHostVar->data[toHostVar->indx++] = c+'0';

	return;
}

void outhex32_Buffer(ToHost *toHostVar, unsigned long n)
{
	outhex8_Buffer(toHostVar, (char) (n >> 24) );
	outhex8_Buffer(toHostVar, (char) (n >> 16) );
	outhex8_Buffer(toHostVar, (char) (n >> 8) );
	outhex8_Buffer(toHostVar, (char) n);
}

//------------------------------------------------------------------
// EZUSB�� ������� ������ polling�� ������� �ʰ� �׳� data�� EZUSB���ٰ�
// write �� �� �־��µ� Arduino Due�� Native USB�� Virtual COM���� ����� 
// ��쿡�� ������ write �� ���� ����. ���� ���� �Ǿ ���ư��� PC ��
// application�� ���ư��� ���� ��쿡�� lock �ɸ��Ƿ� USB buffer�� �ٷ�
// ���� ���� ������ software buffer���ٰ� ������ data�� ��Ƹ� ����. �׷�����
// �����ؾ� �� ������ �Ǹ� USB�� tx_buffer�� Ȯ���Ǿ� �ִ����� Ȯ���ϰ�
// ���� Ȯ���� �Ǿ��ٸ� tx_buffer���ٰ� write �ϸ� �ǰ� ���� tx_buffer��
// Ȯ���� ���� �ʾҴٸ� �׳� �ƹ��ϵ� ���ϰ� �Ѿ�� �ȴ�. �׸��� 
// tx_buffer�� Ȯ���ɶ����� software buffer�� ���̻� ä������ �ȴ´�.
// �Ʒ��� �Լ��� PC������ �����ؾ� �� data�� ���� ���ŵǾ��� ��� 
// �����ؾ� �� ������ data�� ����� ���� ���� software buffer���ٰ�
// ä���ְ� �����ؾ� �� ������ �Ǿ��� �� USB�� tx_buffer���ٰ� 
// write �ϴ� ������ �ϴ� �Լ��̴�.
// Data�� hex encoding�� ����ϰ� Ư�� ����Ÿ�Ӷ��� data�� ����
// ����Ÿ�Ӷ��� data�� �����ϱ� ���� �����ڷν� 'T'�� 'Q'�� ����Ѵ�.
// �׸��� �����ؾ� �� data�� ���������� 'Z'�� ����Ѵ�. ���� 
// ������ �� data�� 2��, Modulus number�� 2�� ��� data�� ���´� 
// ������ ����.
// T_XXXXXXXXXX_XXXXXXXXXX_Q_T_XXXXXXXXXX_XXXXXXXXXX_Q_Z
// ���� ���� data�� ���� n, modulus number�� mod ��� ���� ��
// �ѹ��� �����ϴ� data�� ���Ե� byte�� ���� ������ ���� ���ȴ�.
// 
// # of bytes = (n*10+2)*mod+1
//
// ���⼭ 10�� �ϳ��� data�� hex encoding �ǰ� ������ byte ��
// 2�� 'T'�� 'Q'�� ���� byte ��
// �������� 1�� 'Z'�� ���� byte ��
//
// ���� High Speed CDC�� ��� ������ ���� ���ǵǾ� �����Ƿ�
// #define UDI_CDC_DATA_EPS_HS_SIZE    512
//
// ���� ���Ŀ� ���� ���� �� byte ���� 512���� 1���� 511���� ����
// ���� �����Բ� mod�� ����ؾ� �Ѵ�. ���� �̰��� �������� �����
// ���� 
// 
//  mod = 510/(n*10+2) <== ���������̹Ƿ� ���� ������ ���´�.
//
// ���� ��� n=2�� ��� 
// 
// mod = 510/(2*10+2) = 23.18 => mod = 23
//------------------------------------------------------------------
void ProcessToHostBlock()
{
  unsigned int i, n;
  signed long temp_int32;
  char type;
  static int flag = 0;

  n = toSimulink.NumOfToHostBlock;

  if(toSimulink.UpdateDoneFlag)
  {
    if(toSimulink.SendHoldFlag==0)
    {
      putcToBuffer(&toSimulink, 'T');  // 
      for(i=0;i<n;i++)
      {
        //---------------------------------------------------------
        // For-loop�� ���鼭 �ش� block�� �Է��� ������ �Ǿ�
        // �ִ� ���¶�� Host������ �����Ѵ�.
        // ������ �Ǿ������� ���δ� UpdateDoneFlag[]�� �˻��Ѵ�.
        //---------------------------------------------------------
        //TypeAndChannel[i]�� ����
        // ���� 4-bit�� channel number, ���� 4-bit�� data type.
        // data type�� ���� ������ �Ʒ��� �־��� �ִ�.
        //---------------------------------------------------------

        if(GetBit(toSimulink.UpdateDoneFlag,i))  // ������ �� ���
        {
          outhex8_Buffer(&toSimulink, toSimulink.TypeAndChannel[i]); // channel number ����

          type = (toSimulink.TypeAndChannel[i] >> 4) & 0x0F;  // data type�� �˾Ƴ���.

          //--------------------------------------------
          // data type
          //--------------------------------------------
          // SS_DOUBLE  =  0,    64-bit double �� data 
          // SS_SINGLE  =  1,    /* real32_T  */
          // SS_INT8    =  2,    /* int8_T    */
          // SS_UINT8   =  3,    /* uint8_T   */
          // SS_INT16   =  4,    /* int16_T   */
          // SS_UINT16  =  5,    /* uint16_T  */
          // SS_INT32   =  6,    /* int32_T   */
          // SS_UINT32  =  7,    /* uint32_T  */
          // SS_BOOLEAN =  8     /* boolean_T */
          //--------------------------------------------
          switch(type)
          {
            case 0:  // double
                outhex32_Buffer(&toSimulink, *(unsigned long*)(toSimulink.DataPtr[i])); // double �� data�� ���� 4-byte ���� ����
                
                //---------------------------------------------------------------------------
                // ���� 4 byte�� ������ ���� ���� 4 byte�� ������ ���� �ٽ� type�� channel 
                // ������ �����Ѵ�. ���⼭ type�� double�� data�� ��Ÿ���� 0�� �ƴ϶� 9�� 
                // ����ϱ�� �Ѵ�. ���⼭ 9�� �ǹ��ϴ� ���� double�� data�� �ι�° data��� 
                // ���� �ǹ��Ѵ�. ������������ �� �����κ��� data�� �̾�ٿ� ��� �Ǵ� ���� 
                // �� �� �ְų� Ȥ���� type �� 0 �̿��µ� �� ������ 9�� ������ ������ 
                // ���� ó���� �� �ֵ��� �Ѵ�.
                //-------------------------------------------------------------------------
                type = toSimulink.TypeAndChannel[i];
                type = type & 0x0F;  // ���� 4 bit�� clear ��Ű�� ���� 4 bit(channel ����)�� �״�� ����. 
                type = type | 0x90;  // ���� 4 bit�� 9�� ��ġ��Ų��. double �� data�� 10 byte 2���� ������ ������
                                     // �ϹǷ� ���� 4 byte�� 9��� ���� double �� data�� 2��° data�� �ǹ�
                outhex8_Buffer(&toSimulink, type);  // 2��° data�� �ǹ�
                outhex32_Buffer(&toSimulink, *((unsigned long*)(toSimulink.DataPtr[i])+1)); // double �� data�� ���� 4-byte ����
                break;
            case 1:  // signle data
            case 6:  // int 32
            case 7:  // uinit 32
                outhex32_Buffer(&toSimulink, *(unsigned long*)(toSimulink.DataPtr[i])); // channel data ����
                break;
            case 2:  // int 8
                temp_int32 = (signed long)(*(signed char *)(toSimulink.DataPtr[i]));
                outhex32_Buffer(&toSimulink,*(unsigned long*)&temp_int32);
				break;
            case 4:  // int 16
                temp_int32 = (signed long)(*(signed short *)(toSimulink.DataPtr[i]));
                outhex32_Buffer(&toSimulink,*(unsigned long*)&temp_int32);
              break;
            case 3:  // uint 8
                temp_int32 = (unsigned long)(*(unsigned char *)(toSimulink.DataPtr[i]));
                outhex32_Buffer(&toSimulink,*(unsigned long*)&temp_int32);
				break;
            case 5:  // uint 16
                temp_int32 = (unsigned long)(*(unsigned short *)(toSimulink.DataPtr[i]));
                outhex32_Buffer(&toSimulink,*(unsigned long*)&temp_int32);
				break;
            case 8:  // bool
                temp_int32 = (unsigned long)(*(unsigned char *)(toSimulink.DataPtr[i]));
                outhex32_Buffer(&toSimulink, *(unsigned long*)&temp_int32);
                break;
            default:
                    break;
          }

            //--------------------------------------------
            // �����ϰ� ���� Flag resetting. �̷��� �ؾ���
            // ������ �� �����ϰ� ���� Flag=1�� �����ؼ�
            // ���ſ��θ� �Ǵ��� �� �����ϰ� �Ǵ� ���̴�.
            //--------------------------------------------
          ClearBit(toSimulink.UpdateDoneFlag,i);
        }
      }

      putcToBuffer(&toSimulink,'Q');  // Frame End

      toSimulink.ModCntr++;
      toSimulink.ModCntr%=toSimulink.ModNumber;
      if(toSimulink.ModCntr==0)
      {
        putcToBuffer(&toSimulink,'Z');
      }
    }
    
    //--------------------------------------------------------------------------
    // ModCntr=0 �̸� data�� �����ؾ� �� ������ �ǹ��Ѵ�. ���� �� ������
    // tx_buffer�� Ȯ���� �ȵǾ��ִٸ� data�� ������ �ʰ� SendHoldFlag�� true��
    // �����ؼ� buffer�� Ȯ���ɶ������� ���̻� toSimulink�� ä���ִ�
    // ���� ���� �ʵ��� �Ѵ�. PC���� Simulink�� �������� ������ PC ������
    // data�� �о�� �����Ƿ� ���⼭ �����ϴ� ������ �߻��Ѵ�. 
    // PC�� Simulink�� �������� �ʴ��� �̷� ���� u-controller ����
    // application�� Lock �Ǵ� ��찡 �߻��ϸ� �ȵǹǷ� �̸� �ذ��ϱ� ����
    // ��� �Ʒ� �κ��� �߰��ȴ�. �ϴ� SendHoldFlag=true�� �Ǹ� ���̻� 
    // ModCntr�� �������� �����Ƿ� ����ؼ� 0���� ���� �ְ� �ȴ�. ����
    // ���� iteration ���� ����� ������ �ǰ� ���� tx_buffer�� Ȯ���Ǹ�
    // data�� ������ �ȴ�. �׷��� �Ǹ� �ڿ������� SendHoldFlag �� �ٽ� false
    // �� �ǹǷ� �������� ���۵����� ����� �� �ִ�. 
    // SendHoldFlag �� true��� ���� Sending ������ Hold �ϰ� �ִ� ���¸�
    // �ǹ��Ѵ�
    //--------------------------------------------------------------------------

//---------------------------------------------- 
// STM32F303K8�� USART2�� �̿��� Serial�� �� ��� Ȱ��ȭ �� ��
//----------------------------------------------
#if 1
    if(toSimulink.ModCntr==0)  // <-- �����ؾ� �� ������ �ǹ�
    {
		//if(Serial.availableForWrite() > toSimulink.indx)
		{
			//SerialWrite(toSimulink.data, toSimulink.indx);
			HAL_UART_Transmit(&huart2,toSimulink.data,toSimulink.indx,0xffff);
			toSimulink.indx = 0;  // ������ ���̹Ƿ� toSimulink���� data�� 0���� ������ �� ���´�.
			toSimulink.SendHoldFlag = 0; // ���� ������ ���̹Ƿ� SendHoldFlag�� false�� ����
		}
	}
#endif 

    //----------------------------------------------
    // STM32F303K8�� SPI ��ɰ� SpiEzusb�� �� ��� �� ��� Ȱ��ȭ��ų ��.
    //----------------------------------------------
    #if 0
        if(toSimulink.ModCntr==0)  // <-- �����ؾ� �� ������ �ǹ�
        {
        	SpiEzusbWrite(toSimulink.data, toSimulink.indx);
        	PKTEND_Pulse();
   			toSimulink.indx = 0;  // ������ ���̹Ƿ� toSimulink���� data�� 0���� ������ �� ���´�.
   			toSimulink.SendHoldFlag = 0; // ���� ������ ���̹Ƿ� SendHoldFlag�� false�� ����
    	}
    #endif

  }
  return;
}

