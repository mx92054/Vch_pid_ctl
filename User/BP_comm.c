#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SysTick.h"

#include "BP_comm.h"

extern short wReg[] ;

/////////////function
double sigmoid(double v)
{
    return 1 / (1 + exp(-v));
}

double randomWeight(void) //random weight generator between -0.5 ~ 0.5
{
    return ((int)rand() % 100000) / (float)100000 - 1;
}

void createNetWorks(NNET *nnet, int NumberOfLayers, int *NumberOfNeurons)
{
    nnet->numberOfLayers = NumberOfLayers;
    nnet->layers = (LAYER *)malloc(NumberOfLayers * sizeof(LAYER));
    for (int i = 0; i < NumberOfLayers; i++)
    {
        nnet->layers[i].numberOfNeurons = NumberOfNeurons[i];
        nnet->layers[i].neurons = (NEURON *)malloc(NumberOfNeurons[i] * sizeof(NEURON));
    }
}

void init(NNET *nnet, double *inputs)
{
    for (int i = 0; i < nnet->layers[0].numberOfNeurons; i++)
    {
        nnet->layers[0].neurons[i].output = inputs[i];
    }
    for (int i = 1; i < nnet->numberOfLayers; i++)
    {
        for (int j = 0; j < nnet->layers[i].numberOfNeurons; j++)
        {
            nnet->layers[i].neurons[j].weights = (double *)malloc(nnet->layers[i - 1].numberOfNeurons * sizeof(double));
            double input = 0;
            for (int kk = 0; kk < nnet->layers[i - 1].numberOfNeurons; kk++)
            {
                double weight = randomWeight();
                nnet->layers[i].neurons[j].weights[kk] = weight;
                input += nnet->layers[i - 1].neurons[kk].output * weight;
            }
            nnet->layers[i].neurons[j].input = input;
            nnet->layers[i].neurons[j].output = sigmoid(input);
        }
    }
}
void feedforward(NNET *nnet)
{
    for (int i = 1; i < nnet->numberOfLayers; i++)
    {
        for (int j = 0; j < nnet->layers[i].numberOfNeurons; j++)
        {
            double input = 0;
            for (int kk = 0; kk < nnet->layers[i - 1].numberOfNeurons; kk++)
            {
                double weight = nnet->layers[i].neurons[j].weights[kk];
                input += nnet->layers[i - 1].neurons[kk].output * weight;
            }
            nnet->layers[i].neurons[j].input = input;
            nnet->layers[i].neurons[j].output = sigmoid(input);
        }
    }
}

void feedforwardWithiInput(NNET *nnet, double *input)
{
    for (int i = 0; i < nnet->layers[0].numberOfNeurons; i++)
    {
        nnet->layers[0].neurons[i].output = input[i];
    }
    for (int i = 1; i < nnet->numberOfLayers; i++)
    {
        for (int j = 0; j < nnet->layers[i].numberOfNeurons; j++)
        {
            double input = 0;
            for (int kk = 0; kk < nnet->layers[i - 1].numberOfNeurons; kk++)
            {
                double weight = nnet->layers[i].neurons[j].weights[kk];
                input += nnet->layers[i - 1].neurons[kk].output * weight;
            }
            nnet->layers[i].neurons[j].input = input;
            nnet->layers[i].neurons[j].output = sigmoid(input);
        }
    }
}

void backprop(NNET *nnet, double *targets)
{
    //double **Errors= (double**)malloc(nnet->numberOfLayers * sizeof(double*));

    int num = nnet->layers[nnet->numberOfLayers - 1].numberOfNeurons;
    //Errors[nnet->numberOfLayers - 1]=(double*)malloc((num+1)*sizeof(double));
    for (int i = 0; i < num; i++)
    {
        double out = nnet->layers[nnet->numberOfLayers - 1].neurons[i].output;
        nnet->layers[nnet->numberOfLayers - 1].neurons[i].Error = out * (1 - out) * (targets[i] - out);
    }

    for (int i = nnet->numberOfLayers - 1; i >= 0;)
    {
        if (i != 0)
        {
            //	Errors[i - 1] = (double*)malloc(nnet->layers[i - 1].numberOfNeurons * sizeof(double));
            for (int jj = 0; jj < nnet->layers[i - 1].numberOfNeurons; jj++)
            {
                double temp = 0;
                for (int kk = 0; kk < nnet->layers[i].numberOfNeurons; kk++)
                {
                    temp += nnet->layers[i].neurons[kk].weights[jj] * nnet->layers[i].neurons[kk].Error;
                    nnet->layers[i].neurons[kk].weights[jj] = nnet->layers[i].neurons[kk].weights[jj] + ETA * nnet->layers[i].neurons[kk].Error * nnet->layers[i - 1].neurons[jj].output;
                }
                double out = nnet->layers[i - 1].neurons[jj].output;

                nnet->layers[i - 1].neurons[jj].Error = out * (1 - out) * temp;
            }
        }
        i--;
    }
}

void bp_initilize(NNET *nnet)
{
    int num = 3;
    int a[4] = {3, 3, 1};
    double input[] = {3, 2, 1};

    nnet = (NNET *)malloc(sizeof(NNET));
    createNetWorks(nnet, num, a);
    init(nnet, input);
}

void bp_plant_init(Plant *plt)
{
    plt->sys_state0 = 0.0;
    plt->sys_state1 = 0.0;
    plt->delay_state = 0.0;

    plt->in = 0.0;
    plt->out = 0.0;
}

void bp_plant_step(Plant *plt, float u)
{
    float tmp;

    plt->in = u;

    plt->out = plt->delay_state;

    plt->delay_state = 250.0 * plt->sys_state1;

    tmp = ((plt->in - 12.0 * plt->sys_state0) - plt->sys_state1) / 115.0;

    plt->sys_state1 = plt->sys_state0;
    plt->sys_state0 = tmp;
}

//------------------------------------------------------------------------
int rulelist[7][7] = {
    {-3, -3, -2, -2, -1, 0, 1},
    {-3, -3, -2, -1, -1, 0, 1},
    {-2, -2, -2, -1, 0, 1, 1},
    {-2, -2, -1, 0, 1, 2, 2},
    {-1, -1, 0, 1, 1, 2, 2},
    {-1, 0, 1, 2, 2, 2, 3},
    {0, 0, 2, 2, 2, 3, 3}};
/* ******************************************************
 * Desc:控制器初始化
 * Param: Fuzzy_ctl_block
 * Retval: None
 * *******************************************************/
void Fuzzy_ctl_init(Fuzzy_ctl_block *pfuzzy)
{
    pfuzzy->e = pfuzzy->target - pfuzzy->actual;
    pfuzzy->e_pre = 0;
    pfuzzy->de = pfuzzy->e - pfuzzy->e_pre;
}

/* ******************************************************
 * Desc:三角函数
 * Param: Fuzzy_ctl_block
 * Retval: None
 * *******************************************************/
float trimf(short x, short a, short b, short c)
{
    float u;
    if (x >= a && x <= b)
        u = (float)(x - a) / (float)(b - a);
    else if (x > b && x <= c)
        u = (float)(c - x) / (float)(c - b);
    else
        u = 0.0f;

    return u;
}

/* ******************************************************
 * Desc:设置模糊规则
 * Param: Fuzzy_ctl_block
 * Retval: None
 * *******************************************************/
void setRule(Fuzzy_ctl_block *pfuzzy)
{
    int i, j;
    for (i = 0; i < RULE_NUM; i++)
        for (j = 0; j < RULE_NUM; j++)
            pfuzzy->rule[i][j] = rulelist[i][j];
}

/* ******************************************************
 * Desc:模糊控制计算
 * Param: Fuzzy_ctl_block
 * Retval:  输出值
 * *******************************************************/
short fuzzy_step(Fuzzy_ctl_block *pfuzzy, short delta)
{
    float u_e[RULE_NUM];  //误差隶属度
    float u_de[RULE_NUM]; //误差变化率隶属度
    float u_u, fout;      //输出隶属度

    float den, num;

    int u_e_index[3];  //每个输入只激活3个模糊子集
    int u_de_index[3]; //每个输入只激活3个模糊子集

    int i, j, val;
    short start;  //三角函数起点
    short intval; //三角函数间隔

    pfuzzy->e = delta;
    //e模糊化，计算误差隶属度
    start = -1800;
    intval = 450; // =1800/4 ;
    j = 0;
    for (i = 0; i < RULE_NUM; i++)
    {
        u_e[i] = trimf(pfuzzy->e, start, start + intval, start + 2 * intval);
        if (u_e[i] > 0.001 )
            u_e_index[j++] = i;
        start += intval;
    }
    for (; j < 3; j++)
        u_e_index[j] = 0;

    pfuzzy->de = delta - pfuzzy->e_pre;
    pfuzzy->e_pre = delta;
    if (pfuzzy->de < -60)
        pfuzzy->de = -60;
    if (pfuzzy->de > 60)
        pfuzzy->de = 60;
    //de模糊化，计算误差变化率隶属度
    start = -60;
    intval = 15; //=60/4
    j = 0;
    for (i = 0; i < RULE_NUM; i++)
    {
        u_de[i] = trimf(pfuzzy->de, start, start + intval, start + 2 * intval);
        if (u_de[i] > 0.001 )
            u_de_index[j++] = i;
        start += intval;
    }
    for (; j < 3; j++)
        u_de_index[j] = 0;

    den = 0.0f;
    num = 0.0f;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            num += u_e[u_e_index[i]] * u_de[u_de_index[j]] * (float)rulelist[u_e_index[i]][u_de_index[j]];
            den += u_e[u_e_index[i]] * u_de[u_de_index[j]];
        }

    u_u = (num / den) * 130.0f / 3.0f;
    val = (int)u_u;

    //根据推进力曲线，将推力转化到输出电压
    //Voltage(v):2  	3  		4       5       6       7       8       9       10
    //Force(kgf):2.0  3.0  4.5     9.8     21.7    46.3    70.8    97.2    130
    //根据拟合曲线计算
    // vol = 0.0294*f^3 - 7.057*f^2 + 609.4*f + 8100.0
    u_u = (u_u > 0) ? u_u : -u_u;
    fout = u_u * 0.0294f - 7.057f;
    fout = u_u * fout + 609.4f;
    fout = u_u * fout + 8100.0f;

    if (u_u < 2)
        fout = 0.0f;

    if (val > 0)
        val = (int)fout;
    else
        val = -(int)fout;

    if (val > 32767)
        val = 32767;
    if (val < -32767)
        val = -32767;

    pfuzzy->digit_out = val & 0x0000FFFF;
    pfuzzy->digit_out += 0x8000;

    return pfuzzy->digit_out;
}
