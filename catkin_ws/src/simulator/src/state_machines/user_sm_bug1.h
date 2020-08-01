/********************************************************
 *                                                      *
 *                                                      *
 *      user_sm.h			          	*
 *                                                      *
 *							*
 *		FI-UNAM					*
 *		17-2-2019                               *
 *                                                      *
 ********************************************************/

#include <math.h>
#include <stdlib.h>

#define THRESHOLD 20
// State Machine
void user_sm_bug1(float intensity, float *light_values, float *observations, int size, float laser_value, int  dest, int obs ,
					movement *movements  ,int *next_state ,float Mag_Advance ,float max_twist, float light_x, float light_y,
					float robot_x, float robot_y, float robot_theta, float *m, float *xo, float *yo, float *xm, float *ym, int *vc)
{

 int state = *next_state;
 int i;
 int index_menor;
 float menor;
 float qx[1];
 float qy[1];
 float diference_x = light_x - robot_x;
 float diference_y = light_y - robot_y;
 float distance = sqrt((diference_x * diference_x) + (diference_y * diference_y));
 float angle = atan(fabs(diference_y / diference_x));
 float pendiente = *m;
 float radio;
 float inicio;
 float recorrido;
 float tolerancia;

 int vueltacompleta = *vc;

 qx[0] = *xo;
 qy[0] = *yo;
 qx[1] = *xm;
 qy[1] = *ym;

 //Mide la distancia entre el punto menor y la luz
 radio = sqrt(((qx[1]-light_x) * (qx[1]-light_x)) + ((qy[1]-light_y) * (qy[1]-light_y)));
 //Mide la distancia entre el punto de choque y el punto actual
 inicio = sqrt(((qx[0]-robot_x) * (qx[0]-robot_x)) + ((qy[0]-robot_y) * (qy[0]-robot_y)));
 //Mide la distancia entre el punto de choque y el punto menor
 recorrido = sqrt(((qx[0]-qx[1]) * (qx[0]-qx[1])) + ((qy[0]-qy[1]) * (qy[0]-qy[1])));
 //Mide la distancia entre el punto medio y el punto actual
 tolerancia = sqrt(((qx[1]-robot_x) * (qx[1]-robot_x)) + ((qy[1]-robot_y) * (qy[1]-robot_y)));

 if(diference_x > 0 && diference_y < 0){
	 angle = (8*atan(1)) - angle;
 } else{
	 if(diference_x < 0 && diference_y > 0){
		 angle = -angle + (4*atan(1));
	 } else{
		 if(diference_x < 0 && diference_y < 0){
			 angle = angle + (4*atan(1));
		 } else{
			 angle = angle;
		 }
	 }
 }

 printf("intensity %f\n",intensity);
 printf("quantized destination %d\n",dest);
 printf("quantized obs %d\n",obs);

 for(int i = 0; i < 8; i++)
        printf("light_values[%d] %f\n",i,light_values[i]);
 for (int i = 0; i < size ; i++ )
         printf("laser observations[%d] %f\n",i,observations[i]);

 switch ( state ) {

        case 0:
				if (intensity > THRESHOLD){

						*movements=generate_output(STOP,Mag_Advance,max_twist);
						//printf("Present State: %d STOP\n", state);
						printf("\n **************** Reached light source ******************************\n");
						*next_state = 0;

					}
				else{
					if(robot_theta == angle){
						if(obs == 0){
							*movements=generate_output(FORWARD,Mag_Advance,max_twist);
							*next_state = 0;
						}
						else{
							*xo = robot_x;
							*yo = robot_y;
							*xm = robot_x;
							*ym = robot_y;
							*vc = 0;
							*next_state = 1;
						}
					}
					else{
						*movements=generate_output(LEFT,Mag_Advance,(angle-robot_theta));
						*next_state = 0;
					}
				}
				*m = angle;
								break;

				case 1:
				menor = observations[0];
				index_menor = 0;
				for(int i = 1; i < size; i++){
					if(observations[i] <= menor){
						menor = observations[i];
						index_menor = i;
					}
				}
				if(obs != 0){
					if(index_menor == 0){
						if(menor > 0.8*laser_value)
							*next_state = 3;
						else{
							if(menor < 0.65*laser_value)
								*next_state = 4;
							else{
								*movements=generate_output(FORWARD,0.15*Mag_Advance,max_twist);
							}
						}
					}
					else{
						*next_state = 1;
						*movements=generate_output(LEFT,Mag_Advance,0.1*max_twist);
					}
				}
				else{
					*next_state = 2;
				}
				// Calcula la menor destancia
				if(distance < radio){
					*xm = robot_x;
					*ym = robot_y;
				}
				if(inicio < 0.02 && recorrido > 0.02)
					*vc = 1;
				if(vueltacompleta == 1 && tolerancia < 0.02)
					*next_state = 5;

								break;

				case 2:
				menor = observations[0];
				index_menor = 0;
				for(int i = 1; i < size; i++){
					if(observations[i] <= menor){
						menor = observations[i];
						index_menor = i;
					}
				}
				if(index_menor == 3){
					*movements=generate_output(FORWARD,0.15*Mag_Advance,max_twist);
					*next_state = 1;
				}
				else{
					*movements=generate_output(RIGHT,Mag_Advance,0.1*max_twist);
					*next_state = 2;
				}
								break;

				case 3:
				menor = observations[0];
				index_menor = 0;
				for(int i = 1; i < size; i++){
					if(observations[i] <= menor){
						menor = observations[i];
						index_menor = i;
					}
				}
				if(index_menor == 9){
					*movements=generate_output(FORWARD,0.15*Mag_Advance,max_twist);
					*next_state = 1;
				}
				else{
					*movements=generate_output(RIGHT,Mag_Advance,0.1*max_twist);
					*next_state = 3;
				}
								break;

				case 4:
				menor = observations[0];
				index_menor = 0;
				for(int i = 1; i < size; i++){
					if(observations[i] <= menor){
						menor = observations[i];
						index_menor = i;
					}
				}
				if(index_menor == 9){
					*movements=generate_output(BACKWARD,0.15*Mag_Advance,max_twist);
					*next_state = 1;
				}
				else{
					*movements=generate_output(RIGHT,Mag_Advance,0.1*max_twist);
					*next_state = 4;
				}
								break;

				case 5:
				if (intensity > THRESHOLD){

						*movements=generate_output(STOP,Mag_Advance,max_twist);
						//printf("Present State: %d STOP\n", state);
						printf("\n **************** Reached light source ******************************\n");
						*next_state = 0;

					}
				else{
					if(robot_theta == angle){
						*movements=generate_output(FORWARD,Mag_Advance,max_twist);
						*next_state = 0;
					}
					else{
						*movements=generate_output(LEFT,Mag_Advance,(angle-robot_theta));
						*next_state = 5;
					}
				}
								break;

	default:
		//printf("State %d not defined used ", state);
                *movements=generate_output(STOP,Mag_Advance,max_twist);
                *next_state = 0;
                break;


 }

 printf("X inicial (%f)\n",qx[0]);
 printf("Y inicial (%f)\n",qy[0]);
 printf("X menor (%f)\n",qx[1]);
 printf("Y menor (%f)\n",qy[1]);
 printf("Punto de choque y punto actual (%f)\n",inicio);
 printf("Punto de choque y punto menor (%f)\n",recorrido);
 if(vueltacompleta == 1)
 	printf("vuelta completa\n");


 printf("Next State: %d\n", *next_state);


}
