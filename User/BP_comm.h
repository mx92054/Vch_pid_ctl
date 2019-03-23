#ifndef __BP_COMM_H__
#define __BP_COMM_H__

#define ETA 1 //Learning_rate
#define PRECISION 0.00001

//--------------------------------------
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

#endif
