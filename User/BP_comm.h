#ifndef __BP_COMM_H__
#define __BP_COMM_H__

#define ETA 1 //Learning_rate
#define PRECISION 0.00001

#define RULE_NUM 7

//----------------Neuron Control----------------------
typedef struct Neuron
{
    double input;
    double output;
    double *weights;
    double Error;

} NEURON;

//--------------------------------------
typedef struct Layer
{
    int numberOfNeurons;
    NEURON *neurons;
} LAYER;

//--------------------------------------
typedef struct NNet
{
    int numberOfLayers;
    LAYER *layers;
} NNET;

double sigmoid(double v);
double randomWeight(void); //random weight generator between -0.5 ~ 0.5
void createNetWorks(NNET *nnet, int NumberOfLayers, int *NumberOfNeurons);
void init(NNET *nnet, double *inputs);
void feedforward(NNET *nnet);
void feedforwardWithiInput(NNET *nnet, double *input);
void backprop(NNET *nnet, double *targets);
void bp_initilize(NNET *nnet);

typedef struct _Plant
{
    float in;
    float out;
    float sys_state0;
    float sys_state1;
    float delay_state;
} Plant;

void bp_plant_init(Plant *plt);
void bp_plant_step(Plant *plt, float u);

//---------------Fuzzy Control---------------------------------------------------
typedef struct _fuzzy_controller_block
{
    short target; //控制目标
    short actual; //实际值
    short e;      //误差值 范围-1800 ~ 1800
    short e_pre;  //上次误差值
    short de;     //误差变化率 范围 -100 ~ 100
    short emax;   //误差上限
    short demax;  //误差变化率上限
    short umax;   //输出上限
    short out;    //输出值 范围  -130 ~ 130
    short digit_out ;

    int rule[RULE_NUM][RULE_NUM]; //模糊规则
} Fuzzy_ctl_block;

void Fuzzy_ctl_init(Fuzzy_ctl_block *pfuzzy);    //控制器初始化
float trimf(short x, short a, short b, short c); //三角函数
void setRule(Fuzzy_ctl_block *pfuzzy);           //设置模糊规则
short fuzzy_step(Fuzzy_ctl_block *pfuzzy, short delta);        //模糊控制计算

#endif
