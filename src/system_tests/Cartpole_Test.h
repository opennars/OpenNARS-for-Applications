static double velocity = 0.0;
static double angle = -3.1415/2.0;
static double angle_velocity = 0.0;
static double position = 0.0;
static double max_angle_velocity = 0.3;
static Feedback NAR_CP_Left()
{
    double reverse = angle > 0 ? 1 : -1;
    //if(position > 0.0 && position < 1.0)
    {
        angle_velocity -= reverse * 0.2;
    }
    velocity -= 0.1;
    return (Feedback) {0};
}
static Feedback NAR_CP_Right()
{
    double reverse = angle > 0 ? 1 : -1;
    //if(position > 0.0 && position < 1.0)
    {
        angle_velocity += reverse * 0.2;
    }
    velocity += 0.1;
    return (Feedback) {0};
}
static double successes = 0;
static double failures = 0;
void NAR_Cartpole(long iterations)
{
    char initial[] = "                     |\n"
                     "                     |\n"
                     "     -----------     |\n"
                     "                     |\n"
                     "                     |\n";
    int t=0;
    puts(">>NAR CP start");
    NAR_AddOperation("^left", NAR_CP_Left);
    NAR_AddOperation("^right", NAR_CP_Right);
    while(1)
    {
        position += velocity;
        position = MAX(0.0, MIN(1.0, position));
        angle += angle_velocity;
        CLEAR_SCREEN;
        char world[sizeof(initial)];
        memcpy(world, initial, sizeof(initial));
        DRAW_LINE(10+position*5,2,angle,5,(char*) &world,'o');
        puts(world);
        //gravity
        angle_velocity += 0.2*cos(angle);
        //max. velocities given by air density
        if(angle_velocity > max_angle_velocity)
        {
            angle_velocity = max_angle_velocity;
        }
        if(angle_velocity < -max_angle_velocity)
        {
            angle_velocity = -max_angle_velocity;
        }
        //wrap around angle (where angle 1 corresponds to half a circle)
        double PI = 3.1415; //hm why M_PI doesn't work?
        if(angle > PI)
        {
            angle = -PI;
        }
        if(angle < -PI)
        {
            angle = PI;
        }
        if(t++ > iterations && iterations != -1)
        {
            break;
        }
        printf("position=%f, velocity=%f, angle=%f, angleV=%f\nsuccesses=%f, failures=%f, ratio=%f, time=%d\n", position, velocity, angle, angle_velocity, successes, failures, successes/(successes+failures), t);
        velocity = 0.0; //strong friction
        if(fabs(angle-(-PI/2.0)) <= 0.5) //in balance
        {
            NAR_AddInputNarsese("good. :|:");
            successes += 1.0;
        }
        else
        if(angle >= 0 && angle <= PI)
        {
            failures += 1.0;
        }
        char str[20] = {0};
        int encodingInt = (int)((angle+PI)/(2*PI)*8);
        sprintf(str, "%d. :|:", encodingInt);
        NAR_AddInputNarsese(str);
        NAR_AddInputNarsese("good! :|:");
        NAR_Cycles(3);
        fflush(stdout);
        if(iterations == -1)
        {
            SLEEP;SLEEP;SLEEP;
        }
    }
}
