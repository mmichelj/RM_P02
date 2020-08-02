/********************************************************
 *                                                      *
 *                                                      *
 *              sm_bug2.h                            	*
 *                                                      *
 *		Jesus Savage				*
 *		Diego Cordero				*
 *              Miguel Michel                           *
 *		FI-UNAM					*
 *		17-2-2019                                *
 *                                                      *
 ********************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
#define THRESHOLDD 35
#define M_PI 3.14159265358979323846


// State Machine 
int sm_bug2(float a, float b, float c, int *stepCounter, float qx, float qy, float *qx0, float *qy0, float *max_light_intensity, bool *circledFlag, float *observations, int size, float laser_value, float intensity,float *light_values, int  dest,int obs ,movement *movements  ,int *next_state ,float Mag_Advance ,float max_twist)
{

 int state = *next_state;
 int sensor = 0;
 int i;
 int result = 0;
 int sm_follower = 0;
 float max_advance = Mag_Advance;
 //float d = 0.052;
 float d = 0.05;
 float initialPosition[2] = {0,0};
 initialPosition[0]=*qx0;
 initialPosition[1]=*qy0;
 


 printf("Present State: %d \n", state);
 //printf("intensity %f max intensity %f obstacles %d dest %d \n",intensity, *max_light_intensity,obs,dest);
 //printf("\n******** front = %f ******** \n", observations[2]);

 switch ( state ) {

        case 0:
                //Light follower

		if(intensity > THRESHOLDD)
                {

	                movements->twist = 0.0;
 	                movements->advance = 0.0;
	                result = 1;
	                printf("\n **************** Reached light source ******************************\n");
                }
                else
                {
 	                for(i = 1; i < 8; i++) 
 	                {
	                        if( light_values[i] > light_values[sensor]) sensor = i;
 	                }
 	
 	                if(sensor > 4) sensor = -(8 - sensor);	

	                movements->twist = sensor * 3.1315 / 16;
 	                movements->advance = 0.01;
                }
                
                if(obs != 0 || observations[2]<0.052){
                        *next_state=1;
                        *qx0 = qx;
                        *qy0 = qy;
                        *max_light_intensity = intensity;
                }

               // printf("\n******** front = %f ******** \n", observations[2]);
                
                break;



        case 1: 
                //Wall follower
                //get sensor values
                float mleft =observations[4];
                float mfleft = observations[3];
                float mfront = observations[2];
                float mfright = observations[1];
                float mright = observations[0];
                

                //calculate avoidance vector
                float uao[2] = {0,0};
                float uaoAngle = 0;
                uao[0] = (0.07-mleft)*cos(M_PI)+(0.07-mfleft)*cos(M_PI/4+M_PI)+(0.07-mfront)*cos(M_PI/2+M_PI)+(0.07-mfright)*cos(M_PI*3/4+M_PI)+(0.07-mright)*cos(0);
                uao[1] = (0.07-mleft)*sin(M_PI)+(0.07-mfleft)*sin(M_PI/4+M_PI)+(0.07-mfront)*sin(M_PI/2+M_PI)+(0.07-mfright)*sin(M_PI*3/4+M_PI)+(0.07-mright)*sin(0);
                uaoAngle = atan(uao[1]/uao[0]);
                uaoAngle = -uaoAngle;

                printf("\n uao = [%f, %f], uaoAngle = %f \n", uao[0], uao[1], uaoAngle);

                bool front = (mfront > d) ? 0 : 1;
                bool left = (mleft > d) ? 0 : 1;
                bool right = (mright > d) ? 0 : 1;
                bool fleft = (mfleft > d) ? 0 : 1;
                bool fright = (mfright > d) ? 0 : 1;

                printf("\n******** front = %d ******** \n", front);
                printf("******** fleft = %d ******** \n", fleft);
                printf("******** fright = %d ******** \n ", fright);
                printf("******** right = %d ********\n", right);
                printf("******** left = %d ********\n", left);

                //sensor cases
                if(!front && !fleft && !fright){
                        //look for the wall
                        sm_follower = 0;
                } else if(front && !fleft && !fright){
                        //turn left 
                        sm_follower = 1;
                } else if(!front && !fleft && fright){
                        //follow wall
                        sm_follower = 2;
                } else if(!front && fleft && !fright){
                        //look for wall
                        sm_follower = 1;
                } else if(front && !fleft && fright){
                        //turn left
                        sm_follower = 1;
                } else if(front && fleft && !fright){
                        //turn left
                        sm_follower = 1;
                } else if(front && fleft && fright){
                        //turn left
                        sm_follower = 1;
                } else if(!front && fleft && fright){
                        //look for wall
                        sm_follower = 1;
                }

                //wall follower state machine

                switch(sm_follower){
                        case 0:
                        //look for wall
                         movements->twist = -max_twist;
 	                 movements->advance = max_advance*0.4;
                         printf("sm_follower case 0");
                        break;
                        case 1:
                        //turn left
                         movements->twist = uaoAngle;
 	                 movements->advance = 0;
                         printf("sm_follower case 1 %f", uaoAngle);
                        break;
                        case 2:
                        //follow wall
                         movements->twist = uaoAngle;
 	                 movements->advance = max_advance/4;
                         printf("sm_follower case 2 %f", uaoAngle);
                        break;
                }

                //bug 1 condition
                
                /* TODO
                * Para cada punto: calcular a*qx+b*qy = cActual donde qx y qy son la posición actual del robot
                * Obtener el error e = c-cActual.
                * Si |e| < 0.01, entonces el robot está sobre la misma linea, por lo tanto salir
                */
               //printf("ROBOT INI qx0 = %f qy0 = %f", initialPosition[0], initialPosition[1]);
               
                if(*stepCounter < 15){
                        *stepCounter = *stepCounter + 1;
                        printf("\n stepcounter = %d", *stepCounter);
                } else{
                        float cActual = a*qx + b*qy;
                        float e = c-cActual;
                        printf("\n c = %f cActual = %f e = %f", c, cActual, e);

                        if(e>-0.25 && e<0.25 && intensity > *max_light_intensity){
                                printf("%f",e);
                                *next_state = 0;
                                *stepCounter = 0;
                                movements->twist = M_PI/2;
 	                        movements->advance = 0.01;
                        }
                }

                
                
                break;  
 }

 printf("\nNext State: %d \n", *next_state);
 return result;

}



                 
