static double velocity = 0.0;
static double angle = 0.0;
static double angle_velocity = 0.0;
static double position = 0.0;
static double max_angle_velocity = 0.1;
static void NAR_CP_Left()
{
    angle_velocity = 0.05;
    position -= 0.1;
}
static void NAR_CP_Right()
{
    angle_velocity -= 0.05; //TODO make it an increment
    position += 0.1;
}
static double successes = 0;
static double failures = 0;
void NAR_Cartpole(long iterations)
{
    int t=0;
    NAR_INIT();
    puts(">>NAR CP start");
    NAR_AddOperation(Narsese_AtomicTerm("^left"), NAR_CP_Left); 
    NAR_AddOperation(Narsese_AtomicTerm("^right"), NAR_CP_Right); 
    while(1)
    {
        angle += angle_velocity;
        position += velocity;
        //simplified gravity
        if(angle < 0)
        {
            angle_velocity -= 0.01;
        }
        if(angle > 0)
        {
            angle_velocity += 0.01;
        }
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
        if(angle > 1.0)
        {
            angle = -1.0;
            failures += 1.0;
            NAR_AddInputNarsese("good. :|: {0.0 0.9}");
            puts("WRAPAROUND");
        }
        if(angle < -1.0)
        {
            angle = 1.0;
            failures += 1.0;
            NAR_AddInputNarsese("good. :|: {0.0 0.9}");
            puts("WRAPAROUND");
        }
        if(t++ > iterations && iterations != -1)
        {
            break;
        }
        fputs("\033[1;1H\033[2J", stdout); //POSIX clear screen
        printf("position=%f, velocity=%f, angle=%f, angleV=%f\nsuccesses=%f, failures=%f, ratio=%f, time=%d\n", position, velocity, angle, angle_velocity, successes, failures, successes/(successes+failures), t);
        if(fabs(angle) < 0.1) //in balance
        {
            NAR_AddInputNarsese("good. :|:");
            successes += 1.0;
        }
        else //not in balance
        {
            failures += 1.0;
        }
        char PositionAndVelocity[10] = "11PV. :|:";
        if(position < 0)
        {
            PositionAndVelocity[1] = '0';
        }
        if(position > 0)
        {
            PositionAndVelocity[1] = '2';
        }
        if(velocity > 0)
        {
            PositionAndVelocity[2] = '2';
        }
        if(velocity < 0)
        {
            PositionAndVelocity[2] = '0';
        }
        NAR_AddInputNarsese(PositionAndVelocity);
        NAR_Cycles(5);
        char AngleAndAngleVelocity[10] = "11AV. :|:";
        if(angle < 0)
        {
            AngleAndAngleVelocity[0] = '0';
        }
        if(angle > 0)
        {
            AngleAndAngleVelocity[0] = '2';
        }
        if(angle_velocity > 0)
        {
            AngleAndAngleVelocity[1] = '2';
        }
        if(angle_velocity < 0)
        {
            AngleAndAngleVelocity[1] = '0';
        }
        NAR_AddInputNarsese(AngleAndAngleVelocity);
        NAR_AddInputNarsese("good! :|:");
        fflush(stdout);
        if(iterations == -1)
        {
            nanosleep((struct timespec[]){{0, 20000000L}}, NULL); //POSIX sleep
        }
    }
}
