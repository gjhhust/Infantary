/**********************************************************************************************************
 * @�ļ�     Graphics_Send.c
 * @˵��     ����ϵͳͼ�η���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2021.4
**********************************************************************************************************/
#include "main.h"
#define CAP_GRAPHIC_NUM 9			//�������ݵ���ͼ����ʾϸ�ָ���
extern unsigned char JudgeSend[30];
extern JudgeReceive_t JudgeReceive;
extern F405_typedef F405;
int pitch_change_flag;
int buff_change_flag;
int cap_percent_change_flag;
int vol_change_array[CAP_GRAPHIC_NUM];
int Buff_flag;

float last_cap_vol;
/**********************************************************************************************************
*�� �� ��: JudgementCustomizeGraphics
*����˵��: ����ϵͳ�Զ���ͼ�η��ͺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
ext_student_interactive_header_data_t custom_grapic_draw;			//�Զ���ͼ�����
//װ��ͼ��
void JudgementCustomizeGraphics(int Op_type)
{
		custom_grapic_draw.data_cmd_id=0x0104;//�����߸�ͼ�Σ�����ID����ѯ����ϵͳ�ֲᣩ

		custom_grapic_draw.sender_ID=JudgeReceive.robot_id;//������ID�������˶�ӦID
		if(JudgeReceive.robot_id == 103)
				custom_grapic_draw.receiver_ID = 0x0167;
		if(JudgeReceive.robot_id == 104)
				custom_grapic_draw.receiver_ID = 0x0168;
		if(JudgeReceive.robot_id == 105)
				custom_grapic_draw.receiver_ID = 0x0169;
		if(JudgeReceive.robot_id == 3)
				custom_grapic_draw.receiver_ID = 0x0103;	
		if(JudgeReceive.robot_id == 4)
				custom_grapic_draw.receiver_ID = 0x0104;
		if(JudgeReceive.robot_id == 5)
				custom_grapic_draw.receiver_ID = 0x0105;

/*********************************�Զ���ͼ������***********************************/
		referee_data_load_Graphic(Op_type);
}

/**********************************************************************************************************
*�� �� ��: referee_data_load_shootUI
*����˵��: ͼ�����ݰ�װ�����ͼ��
*��    ��: �������ͣ������˵ȼ�
*�� �� ֵ: ��
**********************************************************************************************************/
short bias_x[7] = {0,15,13,12,11,10,9};
short bias_y[7] = {0,113,118,123,128,133,138};
short bias_r = 100;
short screen_bias = -20;
float XX = 0.58,YY = 0.15;
void referee_data_load_shootUI(uint8_t operate_type,uint8_t robot_level)
{
	/*******************************������ֱ�����Բ*********************************/
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=operate_type;//ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=2;//ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=0;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=0;//��ɫ
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=1;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH/2 + bias_x[0];
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2 + bias_y[0];
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius= 2 + bias_r;

/*******************************ͼ1*********************************/
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 1;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=operate_type;//ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=0;//ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=0;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=1;//��ɫ
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=1;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=SCREEN_LENGTH/2 - bias_x[1] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=SCREEN_WIDTH/2 - bias_y[1];
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x  =SCREEN_LENGTH/2 + bias_x[1] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y  =SCREEN_WIDTH/2 - bias_y[1];
		custom_grapic_draw.graphic_custom.grapic_data_struct[1].radius=0;

/*******************************ͼ2*********************************/
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 2;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=operate_type;//ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=0;//ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=0;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=2;//��ɫ
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=1;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=SCREEN_LENGTH/2 - bias_x[2] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=SCREEN_WIDTH/2 - bias_y[2];
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x  =SCREEN_LENGTH/2 + bias_x[2] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y  =SCREEN_WIDTH/2 - bias_y[2];
		custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=0;

/*******************************ͼ3*********************************/
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 3;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=operate_type;//ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=0;//ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=0;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=3;//��ɫ
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=1;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=SCREEN_LENGTH/2 - bias_x[3] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=SCREEN_WIDTH/2 - bias_y[3];
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_x  =SCREEN_LENGTH/2 + bias_x[3] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_y  =SCREEN_WIDTH/2 - bias_y[3];
		custom_grapic_draw.graphic_custom.grapic_data_struct[3].radius=0;

/*******************************ͼ4*********************************/
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[0] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[1] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[2] = 4;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].operate_tpye=operate_type;//ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_tpye=0;//ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].layer=0;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=4;//��ɫ
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].width=1;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_x=SCREEN_LENGTH/2 - bias_x[4] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_y=SCREEN_WIDTH/2 - bias_y[4];
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_x  =SCREEN_LENGTH/2 + bias_x[4] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_y  =SCREEN_WIDTH/2 - bias_y[4];
		custom_grapic_draw.graphic_custom.grapic_data_struct[4].radius=0;

/*******************************ͼ5*********************************/
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[0] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[1] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[2] = 5;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].operate_tpye=operate_type;//ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_tpye=0;//ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].layer=0;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].color=5;//��ɫ
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].width=1;
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_x=SCREEN_LENGTH/2 - bias_x[5] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_y=SCREEN_WIDTH/2 - bias_y[5];
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_x  =SCREEN_LENGTH/2 + bias_x[5] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_y  =SCREEN_WIDTH/2 - bias_y[5];
		custom_grapic_draw.graphic_custom.grapic_data_struct[5].radius=0;

/*******************************ͼ6*********************************/
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[0] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[1] = 0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[2] = 6;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].operate_tpye=operate_type;//ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_tpye=0;//ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].layer=0;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].color=6;//��ɫ
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].width=1;
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_x=SCREEN_LENGTH/2 - bias_x[6] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_y=SCREEN_WIDTH/2 - bias_y[6];
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_x  =SCREEN_LENGTH/2 + bias_x[6] + screen_bias;
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_y  =SCREEN_WIDTH/2 - bias_y[6];
		custom_grapic_draw.graphic_custom.grapic_data_struct[6].radius=0;
}

/**********************************************************************************************************
*�� �� ��: referee_data_load_NumberUI
*����˵��: ͼ�����ݰ�װ�����֣�����/�ַ�/���Σ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void referee_data_load_NumberUI(void)
{
	static unsigned int pitch = 0;
	pitch = F405.Gimbal_100 * 10;
	/*******************************Pitch������*********************************/
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 0;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 3;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 1;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_Add;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=5;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=3;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Pink;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_angle=20;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_angle=3;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=4;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=0.7 * SCREEN_LENGTH;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=0.7 * SCREEN_WIDTH;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].radius = pitch & 0x03ff;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x  = (pitch >> 10) & 0x07ff;
	custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y  = (pitch >> 21) & 0x07ff;
}
/**********************************************************************************************************
*�� �� ��: referee_data_load_Graphic
*����˵��: ͼ�����ݰ�װ��ͼ��
*��    ��: ��������
*�� �� ֵ: ��
**********************************************************************************************************/
extern float AD_actual_value;		//����ʵ�ʵ�ѹ
char Voltage_array[CAP_GRAPHIC_NUM] = {0,0,0,0,0,0,0,0,0};	//����ÿ1V���Ը�һλ��1����������һ���Ǵ�14V~22V
int last_Voltage_array[CAP_GRAPHIC_NUM] = {0,0,0,0,0,0,0,0,0};	//��¼�ϴθ���
int Voltage_change[CAP_GRAPHIC_NUM] = {0,0,0,0,0,0,0,0,0};			//��¼�Ƿ��и��ӱ仯
int Color_array[CAP_GRAPHIC_NUM] = {Yellow,Yellow,Yellow,Yellow,Yellow,Yellow,Yellow,Yellow,Yellow};//������ɫ
float g_pos_x[CAP_GRAPHIC_NUM] = {0.57,0.34,0.4,0.52,0.34,0.42,0.62,0.5,0.42};
float g_pos_y[CAP_GRAPHIC_NUM] = {0.65,0.15,0.8,0.1,0.1,0.15,0.1,0.8,0.1};
/*��׼��ƫ����*/
int AIM_bias_x = 0;
int AIM_bias_y = 0;
int placece_x[13]={0,   	100,  20,  20, 	20,  40,  20,  20,  20,  40,  20,  20,  20};
int placece_y[14]={-80,-320,-80,-100,-120,-140,-160,-180,-200,-220,-240,-260,-280,-300};
/*������ƫ����*/
int Road_bias_x = 0;
int Road_bias_y = 0;

#define PACK_NUM 5
void referee_data_load_Graphic(int Op_type)
{
	static int pack_tick = 0;			//���ݰ�������
	static unsigned int pitch = 0;
//	static float Start_Place_x[CAP_GRAPHIC_NUM] = {0.388,0.41,0.432,0.454,0.476,0.498,0.52,0.542,0.564};
//  static float Start_Place_y = 0.02;
//	static float End_Place_x[CAP_GRAPHIC_NUM] = {0.398,0.42,0.442,0.464,0.486,0.508,0.53,0.552,0.574};
//	static float End_Place_y = 0.04;
	static int i;
	int packed_tick = 0;							//װ��������
	/*��ʼ����������������ͼ��*/
	if(Op_type == Op_Init)
	{
		switch(pack_tick % PACK_NUM)
		{
			case 0:
//				/*******************************14V*********************************/
//VOL_1:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].graphic_name[2] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].color=Color_array[0];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].start_x=Start_Place_x[0] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].end_x=End_Place_x[0] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].end_y=End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;
//				/*******************************15V*********************************/
//VOL_2:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].graphic_name[2] = 1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].color=Color_array[1];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].start_x=Start_Place_x[1] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].end_x  =End_Place_x[1] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].end_y  =End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;
//				/*******************************16V*********************************/
//VOL_3:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].graphic_name[2] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].color=Color_array[2];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].start_x=Start_Place_x[2] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].end_x  =End_Place_x[2] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].end_y  =End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;
//				/*******************************17V*********************************/
//VOL_4:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].graphic_name[2] = 3;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].color=Color_array[3];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].start_x=Start_Place_x[3] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].end_x  =End_Place_x[3] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].end_y  =End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;
//				/*******************************18V*********************************/
//VOL_5:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].graphic_name[2] = 4;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].color=Color_array[4];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].start_x=Start_Place_x[4] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].end_x  =End_Place_x[4] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].end_y  =End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?4:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;
//				/*******************************19V*********************************/
//VOL_6:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].graphic_name[2] = 5;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].color=Color_array[5];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].start_x=Start_Place_x[5] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].end_x  =End_Place_x[5] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].end_y  =End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?5:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;
//				/*******************************20V*********************************/
//VOL_7:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].graphic_name[2] = 6;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].color=Color_array[6];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].start_x=Start_Place_x[6] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].end_x  =End_Place_x[6] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].end_y  =End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?6:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;
				break;
			case 1:
//				/*******************************21V*********************************/
//VOL_8:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].graphic_name[2] = 7;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].color=Color_array[7];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].start_x=Start_Place_x[7] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].end_x  =End_Place_x[7] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].end_y  =End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?0:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;
//				/*******************************22V*********************************/
//VOL_9:	custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].graphic_name[1] = 2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].graphic_name[2] = 8;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].layer=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].color=Color_array[8];
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].width=20;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].start_x=Start_Place_x[8] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].start_y=Start_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].end_x  =End_Place_x[8] * SCREEN_LENGTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].end_y  =End_Place_y * SCREEN_WIDTH;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?1:packed_tick].radius=0;
//				if(Op_type == Op_Change) goto CONT;			
				/*******************************pitch�Ƕȸ���*********************************/
PITCH_:	pitch = F405.Gimbal_100 * 10;
				/*******************************Pitch ������*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].graphic_name[1] = 3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].graphic_name[2] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].graphic_tpye=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].layer=3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].color=Pink;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].start_angle=20;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].end_angle=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].width=4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].start_x=0.65 * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].start_y=0.65 * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].radius = pitch & 0x03ff;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].end_x  = (pitch >> 10) & 0x07ff;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?2:packed_tick].end_y  = (pitch >> 21) & 0x07ff;
				if(Op_type == Op_Change) goto CONT_2;
				
				/*******************************���ݵ�ѹ ����*********************************/
CAP_PERCENT:				
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].graphic_name[1] = 3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].graphic_name[2] = 22;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].graphic_tpye=6;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].layer=3;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].color=Pink;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].start_angle=20;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].end_angle=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].width=4;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].start_x=XX * SCREEN_LENGTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].start_y=YY * SCREEN_WIDTH;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].radius = (int)(AD_actual_value*1000) & 0x03ff;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].end_x  = ((int)(AD_actual_value*1000) >> 10) & 0x07ff;
				custom_grapic_draw.graphic_custom.grapic_data_struct[Op_type==Op_Init?3:packed_tick].end_y  = ((int)(AD_actual_value*1000) >> 21) & 0x07ff;
				if(Op_type == Op_Change) goto CONT_1;
				break;
			case 2:
//				/*******************************ƽ�� ����*********************************/
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH * 0.46 + Road_bias_x;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2 * 0.9 + Road_bias_y;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=SCREEN_LENGTH * 0.298 + Road_bias_x;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius = 0;

//				/*******************************ƽ�� �ҳ���*********************************/
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=2;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=SCREEN_LENGTH * 0.5 + Road_bias_x;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=SCREEN_WIDTH/2 * 0.9 + Road_bias_y;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x  =SCREEN_LENGTH * 0.658 + Road_bias_x;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y  =0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[1].radius=0;

//				/*******************************�������ݿ���*********************************/
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 6;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=1;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Yellow;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_angle=0;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=5;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=SCREEN_LENGTH * 0.38;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=SCREEN_WIDTH * 0.008;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x  =SCREEN_LENGTH * 0.582;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y  =SCREEN_WIDTH * 0.052;
//				custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=0;
				
				/*******************************���Բ��*********************************/
Buff:		custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 7;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Red_Blue;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=2;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=SCREEN_LENGTH/2 + bias_x[0] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=SCREEN_WIDTH/2 + bias_y[0] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_x=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_y=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].radius=Buff_flag?bias_r:0;
				if(Op_type == Op_Change) goto CONT_3;
				/*�������������������*/
				/*********************************����*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[2] = 8;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_x=SCREEN_LENGTH/2 - placece_x[0] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_y=SCREEN_WIDTH/2 + placece_y[0] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_x  =SCREEN_LENGTH/2 - placece_x[0] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_y  =SCREEN_WIDTH/2 + placece_y[1] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].radius=0;
				
				/*********************************����0���м����*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[2] = 9;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_x=SCREEN_LENGTH/2 - placece_x[1] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_y=SCREEN_WIDTH/2 + placece_y[2] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_x  =SCREEN_LENGTH/2 + placece_x[1] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_y  =SCREEN_WIDTH/2 + placece_y[2] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].radius=0;
				
				/*********************************����1*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[2] = 10;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_x=SCREEN_LENGTH/2 - placece_x[2] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_y=SCREEN_WIDTH/2 + placece_y[3] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_x  =SCREEN_LENGTH/2 + placece_x[2] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_y  =SCREEN_WIDTH/2 + placece_y[3] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].radius=0;
				
				break;	
			case 3:
				/*********************************����2*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 11;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH/2 - placece_x[3] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2 + placece_y[4] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x  =SCREEN_LENGTH/2 + placece_x[3] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y  =SCREEN_WIDTH/2 + placece_y[4] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=0;
			
				/*********************************����3*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 12;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=SCREEN_LENGTH/2 - placece_x[4] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=SCREEN_WIDTH/2 + placece_y[5] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x  =SCREEN_LENGTH/2 + placece_x[4] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y  =SCREEN_WIDTH/2 + placece_y[5] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].radius=0;

				/*********************************����4*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 13;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=SCREEN_LENGTH/2 - placece_x[5] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=SCREEN_WIDTH/2 + placece_y[6] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x  =SCREEN_LENGTH/2 + placece_x[5] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y  =SCREEN_WIDTH/2 + placece_y[6] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=0;

				/*********************************����5*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_name[2] = 14;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_x=SCREEN_LENGTH/2 - placece_x[6] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].start_y=SCREEN_WIDTH/2 + placece_y[7] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_x  =SCREEN_LENGTH/2 + placece_x[6] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].end_y  =SCREEN_WIDTH/2 + placece_y[7] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[3].radius=0;

				/*********************************����6*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_name[2] = 15;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_x=SCREEN_LENGTH/2 - placece_x[7] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].start_y=SCREEN_WIDTH/2 + placece_y[8] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_x  =SCREEN_LENGTH/2 + placece_x[7] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].end_y  =SCREEN_WIDTH/2 + placece_y[8] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[4].radius=0;

				/*********************************����7*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_name[2] = 16;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_x=SCREEN_LENGTH/2 - placece_x[8] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].start_y=SCREEN_WIDTH/2 + placece_y[9] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_x  =SCREEN_LENGTH/2 + placece_x[8] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].end_y  =SCREEN_WIDTH/2 + placece_y[9] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[5].radius=0;

				/*********************************����8*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_name[2] = 17;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_x=SCREEN_LENGTH/2 - placece_x[9] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].start_y=SCREEN_WIDTH/2 + placece_y[10] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_x  =SCREEN_LENGTH/2 + placece_x[9] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].end_y  =SCREEN_WIDTH/2 + placece_y[10] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[6].radius=0;
				break;
			case 4:
				/*********************************����9*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 18;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH/2 - placece_x[10] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2 + placece_y[11] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x  =SCREEN_LENGTH/2 + placece_x[10] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y  =SCREEN_WIDTH/2 + placece_y[11] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=0;
			
				/*********************************����10*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_name[2] = 19;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_x=SCREEN_LENGTH/2 - placece_x[11] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].start_y=SCREEN_WIDTH/2 + placece_y[12] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_x  =SCREEN_LENGTH/2 + placece_x[11] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].end_y  =SCREEN_WIDTH/2 + placece_y[12] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[1].radius=0;

				/*********************************����11*********************************/
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[1] = 1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_name[2] = 20;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=Op_type;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].layer=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].color=Green;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_angle=0;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].width=1;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_x=SCREEN_LENGTH/2 - placece_x[12] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].start_y=SCREEN_WIDTH/2 + placece_y[13] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_x  =SCREEN_LENGTH/2 + placece_x[12] + AIM_bias_x;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].end_y  =SCREEN_WIDTH/2 + placece_y[13] + AIM_bias_y;
				custom_grapic_draw.graphic_custom.grapic_data_struct[2].radius=0;	
				break;
			default:
				break;
		}
		pack_tick++;
	}else if(Op_type == Op_Change)
	{
//		for(i = 0;i<CAP_GRAPHIC_NUM;i++)
//		{
//			if(vol_change_array[i] == Op_Change)
//			{
//				switch(i)
//				{
//					case 0:goto VOL_1;break;
//					case 1:goto VOL_2;break;
//					case 2:goto VOL_3;break;
//					case 3:goto VOL_4;break;
//					case 4:goto VOL_5;break;
//					case 5:goto VOL_6;break;
//					case 6:goto VOL_7;break;
//					case 7:goto VOL_8;break;
//					case 8:goto VOL_9;break;
//					default:break;
//				}
//CONT:		vol_change_array[i] = Op_None;	//����仯��־
//				packed_tick++;
//				if(packed_tick == 7) break;		//װ�������˳����
//			}
//		}
		if(cap_percent_change_flag == Op_Change && packed_tick < 7)
		{
			goto CAP_PERCENT;
CONT_1:cap_percent_change_flag = Op_None;
			packed_tick++;
		}
		if(pitch_change_flag == Op_Change && packed_tick < 7)
		{
			 goto PITCH_;
CONT_2:pitch_change_flag = Op_None;
			 packed_tick++;
		}
		if(buff_change_flag == Op_Change && packed_tick < 7)
		{
				goto Buff;
CONT_3:buff_change_flag = Op_None;	
			 packed_tick++;
		}	
	}
}
/**********************************************************************************************************
*�� �� ��: Graphic_Change_Check
*����˵��: �����û��ͼ�εı䶯�����߽���ͼ��ĳ�ʼ��
*��    ��: ��
*�� �� ֵ: 
**********************************************************************************************************/
int Graphic_Change_Check(void)
{
	int int_AD_actual_value; 	//ȡ��			
	int vol_num; 
	int i;
	int pitch_100;
	static int last_pitch_100;
	static int last_color_array[CAP_GRAPHIC_NUM];
	static int last_Buff_Flag;
	/*�������ְٷֱ�*/
	float cap_vol;
	/*���ڳ�ʼ������ͼ�Σ��糵���ߣ����ߣ��͸�����׼�ߵ�*/
	if(F405.Graphic_Init_Flag == 0)		
	{
		return Op_Init;	//����Init,��һֱ����Add���������ͼ��
	}
	/***********************�������ݱ仯��ⲿ��**************************/
	/*���㳬�����ݵ���ʣ����������*/
	int_AD_actual_value = (int)AD_actual_value;	//��������
	vol_num = LIMIT_MAX_MIN(int_AD_actual_value - 13,CAP_GRAPHIC_NUM,0);	
	cap_vol = AD_actual_value;
	for(i = 0;i < vol_num;i++)
	{
		Voltage_array[i] = 1;		
	}
	for(i = vol_num;i < CAP_GRAPHIC_NUM;i++)
	{
		Voltage_array[i] = 0;		
	}
	for(i = 0;i < CAP_GRAPHIC_NUM;i++)
	{
		if(Voltage_array[i] == 1) Color_array[i] = Yellow;
		if(Voltage_array[i] == 0) Color_array[i] = White;
	}
	/*�����Ƿ��и��ӱ仯*/
	for(i = 0;i < CAP_GRAPHIC_NUM;i++)
	{
		if(Voltage_array[i] != last_Voltage_array[i] || Color_array[i] != last_color_array[i])	//�и��ӱ仯
		{
			vol_change_array[i] = Op_Change;
		}	
	}
	/*��¼���ӱ仯*/
	for(i = 0;i < CAP_GRAPHIC_NUM;i++)
	{
		last_Voltage_array[i] = Voltage_array[i];	
		last_color_array[i] = Color_array[i];
	}
	
	/***********************Pitch�Ƕȱ仯��ⲿ��************************/
	pitch_100 = F405.Gimbal_100;
	if(pitch_100 != last_pitch_100) pitch_change_flag = Op_Change;
	last_pitch_100 = pitch_100;
	
	/***********************Buff״̬�仯��ⲿ��************************/
	if(F405.Gimbal_Flag == Gimbal_Buff_Mode)
		Buff_flag = 1;
	else
		Buff_flag = 0;
	if(Buff_flag != last_Buff_Flag)
	{
			buff_change_flag = Op_Change;
	}
	last_Buff_Flag = Buff_flag;
	
	/*���ݵ�ѹ�ٷֱȼ�ⲿ��*/
	if(last_cap_vol != cap_vol) cap_percent_change_flag = Op_Change;
	last_cap_vol = AD_actual_value;
	
	/*������û�з����仯������б仯�򷵻��޸�ͼ��*/
	if(pitch_change_flag == Op_Change || buff_change_flag == Op_Change || cap_percent_change_flag == Op_Change) return Op_Change;
	for(i = 0;i<CAP_GRAPHIC_NUM;i++)
	{
		if(vol_change_array[i] == Op_Change)
			return Op_Change;
	}

	/*��û�б仯*/
	return Op_None;	//���ؿղ���
}
/**********************************************************************************************************
*�� �� ��: referee_data_pack_handle
*����˵��: ����ϵͳͼ�����ݴ������
*��    ��: uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len
*�� �� ֵ: ��
**********************************************************************************************************/
uint8_t seq = 0;	
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	int i;
	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //����֡����	

	memset(JudgeSend,0,frame_length);  //�洢���ݵ���������
	
	/*****֡ͷ���*****/
	JudgeSend[0] = sof;//����֡��ʼ�ֽ�
	memcpy(&JudgeSend[1],(uint8_t*)&len, sizeof(len));//����֡��data�ĳ���
	JudgeSend[3] = seq;//�����
	Append_CRC8_Check_Sum(JudgeSend,frameheader_len);  //֡ͷУ��CRC8

	/*****��������*****/
	memcpy(&JudgeSend[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****���ݴ��*****/
	memcpy(&JudgeSend[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(JudgeSend,frame_length);  //һ֡����У��CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****�����ϴ�*****/

	USART_ClearFlag(UART4,USART_FLAG_TC);
	for(i=0;i<frame_length;i++)
	{
	  USART_SendData(UART4,JudgeSend[i]);
	  while (USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET); //�ȴ�֮ǰ���ַ��������
	}
}
/**********************************************************************************************************
*�� �� ��: GraphicSendtask
*����˵��: ͼ�η�������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
extern ext_student_interactive_char_header_data_t custom_char_draw;  //�Զ����ַ�����
void GraphicSendtask(void *pvParameters)
{
	 static int tick;
	 static int char_change_state,graphic_change_state;
   while (1) {
    
		tick++;
		if(0==tick%1)	
		{
			/*�ַ���Ӣ��*/
			char_change_state = Char_Change_Check();
			if(char_change_state)			//�����û�б仯��û�б仯�Ͳ�������ʡ����
			{
				JudgementCustomizeChar(char_change_state);
				if(char_change_state != Op_None)
					referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_char_draw,sizeof(custom_char_draw));
			}
			/*��������ͼ��*/
			graphic_change_state = Graphic_Change_Check();
			if(graphic_change_state)
			{
				JudgementCustomizeGraphics(graphic_change_state);
				if(graphic_change_state != Op_None)
					referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw));
			}
		} 

			vTaskDelay(1); 	 
#if INCLUDE_uxTaskGetStackHighWaterMark
		
#endif
    }
}
