#include "QLearner.h"

float Q[nStates][nActions] = {0}; //state, action
float et[nStates][nActions] = {0};
int lastState = 0, lastAction = 0;
float Gamma = 0.8, Lambda = 0.1;

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

int QLearner_Update(int state, float reward, int forcedAction)
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
    double Alpha = MOTOR_BABBLING_CHANCE;
    if(myrand() < (int)(Alpha * MY_RAND_MAX))
    {
        action = myrand() % nActions;
    }
    else
    {
        action = maxk;
    }
    if(forcedAction != -1)
    {
        action = forcedAction;
    }
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
    lastAction = action;
    return lastAction;
}
