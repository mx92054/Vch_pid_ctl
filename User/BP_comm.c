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
    plt->sys_state0 = 0.0f;
    plt->sys_state1 = 0.0f;
    plt->delay_state = 0.0f;

    plt->in = 0.0f;
    plt->out = 0.0f;
}

void bp_plant_step(Plant *plt, float u)
{
    float tmp;

    plt->in = u;

    plt->out = plt->delay_state;

    plt->delay_state = 250.0f * plt->sys_state1;

    tmp = ((plt->in - 12.0f * plt->sys_state0) - plt->sys_state1) / 115.0f;

    plt->sys_state1 = plt->sys_state0;
    plt->sys_state0 = tmp;
}

