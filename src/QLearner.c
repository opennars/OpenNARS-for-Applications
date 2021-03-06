#include "QLearner.h"

float Q[nStates][nActions] = {0}; //state, action
float et[nStates][nActions] = {0};
int lastState = 0, lastAction = 0;
float Gamma = 0.8, Lambda = 0.1, Alpha = 0.1;
float Alpha_decay = 0.9999999999;
float Epsilon_decay = 0.9999999999;
int QLearner_lastAction;

void QLearner_INIT()
{
    for(int i=0; i<nStates; i++)
    {
        for(int k=0; k<nActions; k++)
        {
            Q[i][k] = 0.0f;
            et[i][k] = 0.0f;
        }
    }
}

int QLearner_Update(int state, float reward)
{
    int maxk = 0;
    float maxval = -999999;
    for(int k=0; k<nActions; k++)
    {
        if(Q[state][k] > maxval)
        {
            maxk = k;
            maxval = Q[state][k];
        }
    }
    int action=0;
    Alpha = Alpha * Alpha_decay;
    MOTOR_BABBLING_CHANCE = Epsilon_decay * MOTOR_BABBLING_CHANCE;
    double Epsilon = MOTOR_BABBLING_CHANCE;
    if(myrand() < (int)(Epsilon * MY_RAND_MAX))
    {
        action = myrand() % (BABBLING_OPS + 1);
    }
    else
    {
        action = maxk;
    }
    int lastAction = QLearner_lastAction;
    float DeltaQ = reward+Gamma*Q[state][action]-Q[lastState][lastAction];
    et[lastState][lastAction] = et[lastState][lastAction]+1;
    for(int i=0; i<nStates; i++)
    {
        for(int k=0; k<nActions; k++)
        {
            Q[i][k] = Q[i][k]+Alpha*DeltaQ*et[i][k];
            et[i][k] = Gamma*Lambda*et[i][k];
        }
    }
    lastState = state;
    QLearner_lastAction = action;
    return QLearner_lastAction;
}
