/* 
 * The MIT License
 *
 * Copyright 2020 The OpenNARS authors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


int pX = 5;
int pY = 5;
#define DIRECTION_RIGHT '>'
#define DIRECTION_RIGHT_DOWN 'd'
#define DIRECTION_DOWN '^'
#define DIRECTION_LEFT_DOWN 'b'
#define DIRECTION_LEFT '<'
#define DIRECTION_LEFT_UP 'p'
#define DIRECTION_UP 'v'
#define DIRECTION_RIGHT_UP 'q'
char direction = DIRECTION_RIGHT; //right, right down, down, left down, left, left up, up, right up
bool allowAction = false;

//Angle transition via ^left operator
Feedback NAR_Robot_Left()
{
    if(!allowAction)
        return (Feedback) {0};
    if(direction == DIRECTION_RIGHT)
        direction = DIRECTION_RIGHT_UP;
    else
    if(direction == DIRECTION_RIGHT_UP)
        direction = DIRECTION_UP;
    else
    if(direction == DIRECTION_UP)
        direction = DIRECTION_LEFT_UP;
    else
    if(direction == DIRECTION_LEFT_UP)
        direction = DIRECTION_LEFT;
    else
    if(direction == DIRECTION_LEFT)
        direction = DIRECTION_LEFT_DOWN;
    else
    if(direction == DIRECTION_LEFT_DOWN)
        direction = DIRECTION_DOWN;
    else
    if(direction == DIRECTION_DOWN)
        direction = DIRECTION_RIGHT_DOWN;
    else
    if(direction == DIRECTION_RIGHT_DOWN)
        direction = DIRECTION_RIGHT;
    allowAction = false;
    return (Feedback) {0};
}

//Angle transition via ^right operator
Feedback NAR_Robot_Right()
{
    if(!allowAction)
        return (Feedback) {0};
    if(direction == DIRECTION_RIGHT)
        direction = DIRECTION_RIGHT_DOWN;
    else
    if(direction == DIRECTION_RIGHT_DOWN)
        direction = DIRECTION_DOWN;
    else
    if(direction == DIRECTION_DOWN)
        direction = DIRECTION_LEFT_DOWN;
    else
    if(direction == DIRECTION_LEFT_DOWN)
        direction = DIRECTION_LEFT;
    else
    if(direction == DIRECTION_LEFT)
        direction = DIRECTION_LEFT_UP;
    else
    if(direction == DIRECTION_LEFT_UP)
        direction = DIRECTION_UP;
    else
    if(direction == DIRECTION_UP)
        direction = DIRECTION_RIGHT_UP;
    else
    if(direction == DIRECTION_RIGHT_UP)
        direction = DIRECTION_RIGHT;
    allowAction = false;
    return (Feedback) {0};
}

//The world is composed of worldsizeX * worldsizeY cells
typedef struct
{
    bool wall;
    bool food;
}Cell;
#define worldsizeX 41
#define worldsizeY 21
Cell world[worldsizeX][worldsizeY] = {0}; //+1 for the maze generator

//Draw cells
void Cell_Draw(Cell *cell)
{
    if(cell->wall)
    {
        fputs("\x1B[97;47m#\x1B[0m", stdout);
    }
    else
    {
        if(cell->food)
        {
            fputs("\x1B[32;43m+\x1B[0m", stdout);
        }
        else
        {
            fputs("\x1B[30;43m \x1B[0m", stdout);
        }
    }
}

char Cell_State(int i, int j)
{
    if(i < 0 || j < 0 || i >= worldsizeX || j >= worldsizeY)
    {
        return 'w';
    }
    Cell *cell = &world[i][j];
    if(cell->wall)
    {
        return 'w';
    }
    else
    {
        if(cell->food)
        {
            return 'f';
        }
        else
        {
            return 'o';
        }
    }
}

//Draw the world
void World_Draw()
{
    for(int i=0; i<worldsizeY; i++)
    {
        for(int j=0; j<worldsizeX; j++)
        {
            if(j == pX && i == pY)
            {
                char sdir[2] = { direction, 0 };
                fputs("\x1B[31;43;1m", stdout);
                fputs(sdir, stdout);
                fputs("\x1B[0m", stdout);
            }
            else
            {
                Cell_Draw(&world[j][i]);
            }
        }
        puts("");
    }
}

//A perception of the world
typedef struct
{
    int forward_pX; //position the agent reaches in case that
    int forward_pY; //moving forward is possible
    char viewfield[3];
    bool collision;
    bool reward;
    bool moved;
}Perception;

int irand(int n)
{
    int r, rmax = n*(MY_RAND_MAX/n);
    while((r=myrand()) >= rmax);
    return r / (MY_RAND_MAX/n);
}

void spawnFood(bool good)
{
    int x, y;
    do
    {
        x = irand(worldsizeX);
        y = irand(worldsizeY);
        if(!world[x][y].wall && !world[x][y].food)
        {
            if(good)
                world[x][y].food = true;
            if(!good)
                world[x][y].wall = true;
            break;
        }
    }
    while(1);
}

void overwriteViewfield(int distance, bool* collided, char *original, char newval)
{
    if(newval == 'o' || *collided) //nothing hit the view ray position, or something was already hit
    {
        return;    
    }
    *collided = true; //vision can't go through objects
    if(distance > 2 && newval == 'w') //ultrasonic detection only up to this distance
    {
        return;
    }
    *original = newval;
}

Perception Agent_View()
{
    Perception ret = { .viewfield = {'o', 'o', 'o'} };
    int viewdist = 50;
    bool collided[3] = {0};
    for(int i=1;i<=viewdist; i++)
    {
#define BREAK_ON_DONE if(ret.viewfield[0] == 'f' || ret.viewfield[1] == 'f' || ret.viewfield[2] == 'f') \
                      { \
                          break; \
                      }
        BREAK_ON_DONE //not best practice but fine for examples.. :)
        if(direction == DIRECTION_RIGHT)
        {
            ret.forward_pX = pX + 1;
            ret.forward_pY = pY;
            for(int I=1; I<=i; I++)
            {
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX+I, pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX+i, pY+I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[1], &ret.viewfield[1], Cell_State(pX+i, pY));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX+i, pY-I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX+I, pY-i));
                BREAK_ON_DONE
            }
        }
        if(direction == DIRECTION_RIGHT_UP)
        {
            ret.forward_pX = pX + 1;
            ret.forward_pY = pY + 1;
            for(int I=0; I<i; I++)
            {
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX-I, pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX+I, pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[1], &ret.viewfield[1], Cell_State(pX+i, pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX+i, pY+I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX+i, pY-I));
                BREAK_ON_DONE
            }
        }
        if(direction == DIRECTION_UP)
        {
            ret.forward_pX = pX;
            ret.forward_pY = pY + 1;
            for(int I=1; I<=i; I++)
            {
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX-i, pY+I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX-I, pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[1], &ret.viewfield[1], Cell_State(pX,   pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX+I, pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX+i, pY+I));
                BREAK_ON_DONE
            }
        }
        if(direction == DIRECTION_LEFT_UP)
        {
            ret.forward_pX = pX - 1;
            ret.forward_pY = pY + 1;
            for(int I=0; I<i; I++)
            {
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX-i, pY-I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX-i, pY+I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[1], &ret.viewfield[1], Cell_State(pX-i, pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX-I, pY+i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX+I, pY+i));
                BREAK_ON_DONE
            }
        }
        if(direction == DIRECTION_LEFT)
        {
            ret.forward_pX = pX - 1;
            ret.forward_pY = pY;
            for(int I=1; I<=i; I++)
            {
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX-I, pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX-i, pY-I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[1], &ret.viewfield[1], Cell_State(pX-i, pY));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX-i, pY+I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX-I, pY+i));
                BREAK_ON_DONE
            }
        }
        if(direction == DIRECTION_LEFT_DOWN)
        {
            ret.forward_pX = pX - 1;
            ret.forward_pY = pY - 1;
            for(int I=0; I<i; I++)
            {
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX+I, pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX-I, pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[1], &ret.viewfield[1], Cell_State(pX-i, pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX-i, pY-I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX-i, pY+I));
                BREAK_ON_DONE
            }
        }
        if(direction == DIRECTION_DOWN)
        {
            ret.forward_pX = pX;
            ret.forward_pY = pY - 1;
            for(int I=1; I<=i; I++)
            {
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX+i, pY-I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX+I, pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[1], &ret.viewfield[1], Cell_State(pX,   pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX-I, pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX-i, pY-I));
                BREAK_ON_DONE
            }
        }
        if(direction == DIRECTION_RIGHT_DOWN)
        {
            ret.forward_pX = pX + 1;
            ret.forward_pY = pY - 1;
            for(int I=0; I<i; I++)
            {
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX+i, pY+I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[0], &ret.viewfield[0], Cell_State(pX+i, pY-I));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[1], &ret.viewfield[1], Cell_State(pX+i, pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX+I, pY-i));
                BREAK_ON_DONE
                overwriteViewfield(i, &collided[2], &ret.viewfield[2], Cell_State(pX-I, pY-i));
                BREAK_ON_DONE
            }
        }
    }
    bool outside = ret.forward_pX < 0 || ret.forward_pX >= worldsizeX || 
                   ret.forward_pY < 0 || ret.forward_pY >= worldsizeY;
    if(outside ||  world[ret.forward_pX][ret.forward_pY].wall)
    {
        if(ret.forward_pX != pX || ret.forward_pY != pY)
        {
            ret.collision = true;
        }
        ret.forward_pX = pX;
        ret.forward_pY = pY;
    }
    if(world[pX][pY].food)
    {
        ret.reward = true;
        world[pX][pY].food = false;
        spawnFood(true);
    }
    if(ret.forward_pX != pX || ret.forward_pY != pY)
    {
        ret.moved = true;    
    }
    return ret;
}

int eaten = 0;
int moves = 0;

//Forward move
Feedback NAR_Robot_Forward()
{
    if(!allowAction)
        return (Feedback) {0};
    Perception percept = Agent_View();
    //progress movement
    if(pX != percept.forward_pX || pY != percept.forward_pY)
    {
        moves++;    
    }
    pX = percept.forward_pX;
    pY = percept.forward_pY;
    allowAction = false;
    return (Feedback) {0};
}

void buildRooms()
{
    int roomsize=20;
    for(int x=0; x<worldsizeX; x++)
    {
        for(int y=0; y<worldsizeY; y++)
        {
            if(x%roomsize == 0 || y%roomsize == 0)
            {
                world[x][y].wall = true;
            }
            if(x%roomsize == 0 && y%roomsize == 0) //corner
            {
                for(int k=-2; k<=2; k++) //door size
                {
                    if(x-roomsize/2+k >= 0 && y-roomsize/2+k >= 0 && x-roomsize/2+k < worldsizeX && y-roomsize/2+k < worldsizeY)
                    {
                        world[x-roomsize/2+k][y].wall = false;
                        world[x][y-roomsize/2+k].wall = false;
                    }
                }
            }
        }
    }
}

bool collided = false;
void Agent_Invoke()
{
    Perception percept = Agent_View();
    //Use events for the objects seen left, middle, right instead:
    char narseseL[16] = "l_. :|:";
    char narseseM[16] = "m_. :|:";
    char narseseR[16] = "r_. :|:";
    narseseL[1] = percept.viewfield[0];
    narseseM[1] = percept.viewfield[1];
    narseseR[1] = percept.viewfield[2];
    if(percept.viewfield[1] != 'o')
    {
        NAR_AddInputNarsese(narseseM);
    }
    else
    if(percept.viewfield[0] != 'o')
    {
        NAR_AddInputNarsese(narseseL);
    }
    else
    if(percept.viewfield[2] != 'o')
    {
        NAR_AddInputNarsese(narseseR);
    }
    if(percept.moved && collided)
    {
        collided = false;
        NAR_AddInputNarsese("(! collision). :|:");
    }
    if(percept.viewfield[1] == 'w') //distance or touch sensor
    {
        collided = true;
    }
    if(percept.reward)
    {
        eaten++;
        NAR_AddInputNarsese("eaten. :|:");
    }
    allowAction = true;
    if(collided)
    {
        NAR_AddInputNarsese("(! collision)! :|:");
    }
    else
    {
        NAR_AddInputNarsese("eaten! :|:");
    }
}

void NAR_Robot(long iterations)
{
    MOTOR_BABBLING_CHANCE = 0.3;
    puts(">>NAR Robot start");
    NAR_AddOperation("^left", NAR_Robot_Left);
    NAR_AddOperation("^right", NAR_Robot_Right);
    NAR_AddOperation("^forward", NAR_Robot_Forward);
    buildRooms();
    for(int i=0; i<23; i++) { spawnFood(true); }
    for(int i=0; i<23; i++) { spawnFood(false); }
    long t=0;
    while(1)
    {
        t++;
        if(iterations != -1 && t > iterations)
        {
            break;
        }
        fputs("\033[1;1H\033[2J", stdout); //POSIX clear screen
        World_Draw();
        printf("time=%ld moves=%d move_success_ratio=%f eaten=%d reasonerStep=%ld\n", t, moves, (float) (((float) moves) / ((float) t)), eaten, currentTime);
        Agent_Invoke();
        if(iterations == -1)
        {
            SLEEP; SLEEP; //POSIX sleep
        }
    }
}
