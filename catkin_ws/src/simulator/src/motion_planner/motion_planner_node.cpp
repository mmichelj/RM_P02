/***********************************************
*                                              *
*      motion_planner_node.cpp                 *
*                                              *
*      Jesus Savage                            *
*      Diego Cordero                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 17-2-2019                 *
*                                              *
*                                              *
************************************************/

#include "ros/ros.h"
#include "../utilities/simulator_structures.h"
#include "../utilities/random.h"
#include "motion_planner_utilities.h"
#include "../state_machines/light_follower.h"
#include "../state_machines/sm_avoidance.h"
#include "../state_machines/sm_avoidance_modified.h"
#include "../state_machines/sm_avoidance_destination.h"
#include "../state_machines/sm_destination.h"
#include "../state_machines/user_sm.h"
#include "../state_machines/campos_potenciales.h"
//#include "../state_machines/dijkstra.h"
#include "../state_machines/astar.h"
#include "../state_machines/bfs.h"
#include "../state_machines/dfs.h"
#include "clips_ros/SimuladorRepresentation.h"
#include "../behaviors/oracle.h"
#include "../state_machines/sm_bug1.h"
#include "../state_machines/sm_bug2.h"
#include "../state_machines/user_sm_bug2.h"

int main(int argc ,char **argv)
{
    ros::init(argc ,argv ,"simulator_motion_planner_node");
    ros::NodeHandle n;
    ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub",0, parametersCallback);
    ros::Subscriber sub = n.subscribe("/scan", 10, laserCallback);
    SimuladorRepresentation::setNodeHandle(&n);
    ros::Rate r(20);


    float lidar_readings[512];
    float light_readings[8];
    
    int i;
    int tmp;
    int sensor;
    int est_sig;
    int q_light;
    int q_inputs;
    int flagOnce;
    int flg_finish;
    int mini_sm=1;
    int cta_steps;
    int flg_result;
    int flg_noise=0;
    int vuelta;
    int bug_sm=0;

    int q_inputs2=0;
    int stepCounter;
    
    float result;
    float final_x;
    float final_y;
    float intensity;
    float max_advance;
    float max_turn_angle;
    float noise_advance;
    float noise_angle;

    float pendiente;
    float Xo;
    float Yo;
    float Xm;
    float Ym;
    

    float a = 0;
    float b = 0;
    float c = 0;

    float initialPosition[2] = {0,0};
    float max_light_intensity = 0;
    bool circledOnce = 0;

    float pos_x;
    float pos_y;
    float qx;
    float qy;
    float qx0;
    float qy0;
    float qxdes;
    float qydes;
    float theta;
    
    char path[100];
    char object_name[20];

    /*float pos_history[2][10]={
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };*/

    float pos_history[2]={0,0};
    float diferencia_x=0;
    float diferencia_y=0;
    int cont_similar=0;
    bool obstacle_flag=0;


    movement movements;
    step steps[200];
    step graph_steps[200];

    // it sets the environment's path
    strcpy(path,"./src/simulator/src/data/");

    while( ros::ok()  )
    {
        flagOnce = 1;
        cta_steps = 0;
        mini_sm =1;

        while( params.run )
        {
            // it gets sensory data
            ros::spinOnce();

            if (!params.turtle)
            {
                get_light_values(&intensity,light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

                get_lidar_values(lidar_readings,params.robot_x,
                                 params.robot_y,params.robot_theta,params.turtle); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            }
            else
            {
                get_light_values_turtle(&intensity,light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
                for( i = 0; i < 512; i++)
                    lidar_readings[i] = lasers[i];
            }

            // it quantizes the sensory data
            q_light = quantize_light(light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            
            if(params.noise )
                q_inputs = quantize_laser_noise(lidar_readings,params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            else
                q_inputs = quantize_laser(lidar_readings,params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h


            max_advance = params.robot_max_advance;
            max_turn_angle = params.robot_turn_angle;

            switch ( params.behavior)
            {

            case 1:
                // This function sends light sensory data to a function that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/light_follower.h
                flg_result = light_follower(intensity, light_readings,&movements,max_advance,max_turn_angle);
                if(flg_result == 1) stop();
                break;

            case 2:
                // This function sends light sensory data to an state machine that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_destination.h
                if(flagOnce)
                {
                    est_sig = 1;
                    flagOnce = 0;
                }
                flg_result = sm_destination(intensity,q_light,&movements,&est_sig,params.robot_max_advance,params.robot_turn_angle);
                if(flg_result == 1) stop();

                break;

            case 3:
                // This function sends quantized sensory data to an state machine that avoids obstacles and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance.h
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                sm_avoid_obstacles(q_inputs,&movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle);
                break;

            case 4:
                // This function sends quantized sensory data to an state machine that follows a light source and avoids obstacles
                // and it issues the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance_destination.h
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
                                                    params.robot_max_advance ,params.robot_turn_angle);

                if(flg_result == 1) stop();
                break;

            case 5:
                // This function sends the intensity and the quantized sensory data to a Clips node and it receives the actions that
                // the robot needs to perfom to avoid obstacles and reach a light source according to first order logic
                // It is located in ~/catkin_ws/src/simulator/src/behaviors/oracle.h
                result=oracle_clips(intensity,q_light,q_inputs,&movements,max_advance ,max_turn_angle);
                if(result == 1.0) stop();
                break;


            case 6:
                // it finds a path from the origen to a destination using depth first search
                if(flagOnce)
                {
                    for(i = 0; i < 200; i++) steps[i].node = -1;

                    // it finds a path from the origen to a destination using first search
                    dfs(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
                    print_algorithm_graph (steps);
                    i = 0;
                    final_x = params.light_x;
                    final_y = params.light_y;
                    set_light_position(steps[i].x,steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flagOnce = 0;
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
                else
                {
                    //flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,                                                        //params.robot_max_advance ,params.robot_turn_angle);
                    flg_result = oracle_clips(intensity,q_light,q_inputs,&movements,max_advance ,max_turn_angle);

                    if(flg_result == 1)
                    {
                        if(flg_finish == 1)
                            stop();
                        else
                        {
                            if(steps[i].node != -1)
                            {
                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x,final_y);
                                printf("New Light %d: x = %f  y = %f \n",i,final_x,final_y);
                                flg_finish = 1;
                            }
                        }
                    }
                }

                break;

            case 7:
                /*if(flagOnce)
                {
                    for(i = 0; i < 200; i++)steps[i].node=-1;
                    // it finds a path from the origen to a destination using the Dijkstra algorithm
                    dijkstra(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
                    print_algorithm_graph (steps);
                    i=0;
                    final_x=params.light_x;
                    final_y= params.light_y;
                    set_light_position(steps[i].x,steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flagOnce = 0;
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
                else
                {
                    flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
                                                        params.robot_max_advance ,params.robot_turn_angle);

                    if(flg_result == 1)
                    {
                        if(flg_finish == 1) stop();
                        else
                        {
                            if(steps[i].node != -1)
                            {
                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x,final_y);
                                printf("New Light %d: x = %f  y = %f \n",i,final_x,final_y);
                                flg_finish=1;
                            }
                        }
                    }
                }*/
                break;

            case 8:
                // Light follower
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                flg_result = light_follower(intensity, light_readings,&movements,max_advance,max_turn_angle);
                if(flg_result == 1) stop();
                break;

            case 9:
                //Campos potenciales con evasion de obstaculos

                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }

                //Si no ha cambiado su posición más de .1 en x Y y desde hace 10 iteraciones, salir del obstaculo

                diferencia_x=fabs(pos_history[0]-params.robot_x);
                diferencia_y=fabs(pos_history[1]-params.robot_y);

                pos_history[0]=params.robot_x;
                pos_history[1]=params.robot_y;

                printf("diferencias: %f %f %d",diferencia_x,diferencia_y,cont_similar);

                if(cont_similar<30){
                    if(diferencia_x<0.03 && diferencia_y<0.03){
                    cont_similar++;
                    } else{
                    cont_similar=0;
                    }
                    flg_result = campos_potenciales(intensity, light_readings,&movements,max_advance,max_turn_angle,lidar_readings,params.laser_num_sensors,params.laser_value,params.laser_origin,params.laser_range);
                }

                if(cont_similar>=30 && cont_similar<40){
                    sm_avoid_obstacles_m(q_inputs,&movements,&est_sig,params.robot_max_advance ,params.robot_turn_angle,12);
                    cont_similar++;
                }

                if(cont_similar==40){
                    cont_similar=0;
                    est_sig=0;
                }

                if(flg_result == 1) stop();

                break;

            
            case 10:

                switch(mini_sm)
                {
                    case 1:
                        movements.twist = 0;
                        movements.advance = .1;
                        mini_sm=2;
                    break;
                    case 2:
                        movements.twist = 0;
                        movements.advance = 0;    
                        strcpy(object_name,"a");
                        if(object_interaction(GRASP,object_name))
                            mini_sm=3;
                        else
                            mini_sm=1;
                    break;
                    case 3:
                        movements.twist = 3.1416/2;
                        movements.advance = .1;
                        mini_sm=4;
                    break;
                    case 4:
                        movements.twist = 0;
                        movements.advance = 0; 
                        if(object_interaction(RELEASE,object_name))
                            mini_sm=5;

                    break;
                    case 5:
                        movements.twist = -3.1416/2;
                        movements.advance = 0;
                        mini_sm=6;
                    break;
                    case 6:
                        movements.twist = 0;
                        movements.advance = .1;
                        mini_sm=77;
                    break;
                    case 77:
                        movements.twist = 0;
                        movements.advance = 0;    
                        strcpy(object_name,"b");
                        if(object_interaction(GRASP,object_name))
                            mini_sm=7;
                        else
                            mini_sm=6;
                    break;
                    case 7:
                        movements.twist = 3.1416/2;
                        movements.advance = .1;
                        mini_sm=8;
                    break;
                    case 8:
                        movements.twist = 0;
                        movements.advance = 0; 
                        if(object_interaction(RELEASE,object_name))
                            mini_sm=9;
                    break;
                    case 9:
                        movements.twist = -3.1416/2;
                        movements.advance = .0;
                        mini_sm=10;
                    break;
                    case 10:
                        movements.twist = 0;
                        movements.advance = .1;
                        mini_sm=11;
                    break;
                    case 11:
                        movements.twist = 0;
                        movements.advance = 0;    
                        strcpy(object_name,"c");
                        if(object_interaction(GRASP,object_name))
                            mini_sm=12;
                        else
                            mini_sm=10;
                    break;
                    case 12:
                        movements.twist = 3.1416/2;
                        movements.advance = .1;
                        mini_sm=13;
                    break;
                    case 13:
                        movements.twist = 0;
                        movements.advance = 0; 
                        if(object_interaction(RELEASE,object_name))
                            mini_sm=14;

                    break;
                    case 14:
                        movements.twist = -3.1416/2;
                        movements.advance = .1;
                        mini_sm=15;
                    break;
                    case 15:
                        movements.twist = 0;
                        movements.advance = 0;    
                        strcpy(object_name,"d");
                        if(object_interaction(GRASP,object_name))
                            mini_sm=166;
                        else
                            mini_sm=14;
                    break;
                    case 166:
                        movements.twist = -3.1416/2;
                        movements.advance = .3;
                        mini_sm=16;
                    break;
                    case 16:
                        movements.twist = -3.1416/2;
                        movements.advance = .4;
                        mini_sm=17;
                    break;
                    case 17:
                        movements.twist = 0;
                        movements.advance = 0; 
                        if(object_interaction(RELEASE,object_name))
                            mini_sm=18;

                    break;
                    case 18:
                        movements.twist = -3.1416/2;
                        movements.advance = .1;
                        mini_sm=19;
                    break;
                    case 19:
                        stop();
                    break;
                    
                }
                break;

            case 11:
                //A* con campos potenciales

                if(flagOnce)
                {
                    for(i = 0; i < 200; i++)steps[i].node=-1;
                    // it finds a path from the origen to a destination using the Dijkstra algorithm
                    astar(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
                    print_algorithm_graph (steps);
                    i=0;
                    final_x=params.light_x;
                    final_y= params.light_y;
                    set_light_position(steps[i].x,steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flagOnce = 0;
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
                else
                {

                    /**********************Campos Potenciales*******************************/

                    //Si no ha cambiado su posición más de .1 en x Y y desde hace 10 iteraciones, salir del obstaculo

                    diferencia_x=fabs(pos_history[0]-params.robot_x);
                    diferencia_y=fabs(pos_history[1]-params.robot_y);

                    pos_history[0]=params.robot_x;
                    pos_history[1]=params.robot_y;

                    printf("diferencias: %f %f %d",diferencia_x,diferencia_y,cont_similar);

                    if(cont_similar<30)
                    {
                        if(diferencia_x<0.03 && diferencia_y<0.03){
                        cont_similar++;
                        } else{
                        cont_similar=0;
                        }
                        flg_result = campos_potenciales(intensity, light_readings,&movements,max_advance,max_turn_angle,lidar_readings,params.laser_num_sensors,params.laser_value,params.laser_origin,params.laser_range);
                    }

                    if(cont_similar>=30 && cont_similar<40){
                        sm_avoid_obstacles_m(q_inputs,&movements,&est_sig, params.robot_max_advance * 0.8 ,params.robot_turn_angle * 0.5 ,12);
                        cont_similar++;
                    }

                    if(cont_similar==40){
                        cont_similar=0;
                        est_sig=0;
                    }


                    /*********************************************************************/

                    if(flg_result == 1)
                    {
                        if(flg_finish == 1) stop();
                        else
                        {
                            if(steps[i].node != -1)
                            {
                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x,final_y);
                                printf("New Light %d: x = %f  y = %f \n",i,final_x,final_y);
                                flg_finish=1;
                            }
                        }
                    }
                }

                break;

            case 12:
                //Breadth first search con campos potenciales

                if(flagOnce)
                {
                    for(i = 0; i < 200; i++)steps[i].node=-1;
                    // it finds a path from the origen to a destination using the Dijkstra algorithm
                    bfs(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
                    print_algorithm_graph (steps);
                    i=0;
                    final_x=params.light_x;
                    final_y= params.light_y;
                    set_light_position(steps[i].x,steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flagOnce = 0;
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
                else
                {

                    /**********************Campos Potenciales*******************************/

                    //Si no ha cambiado su posición más de .1 en x Y y desde hace 10 iteraciones, salir del obstaculo

                    diferencia_x=fabs(pos_history[0]-params.robot_x);
                    diferencia_y=fabs(pos_history[1]-params.robot_y);

                    pos_history[0]=params.robot_x;
                    pos_history[1]=params.robot_y;

                    printf("diferencias: %f %f %d",diferencia_x,diferencia_y,cont_similar);

                    if(cont_similar<30)
                    {
                        if(diferencia_x<0.03 && diferencia_y<0.03){
                        cont_similar++;
                        } else{
                        cont_similar=0;
                        }
                        flg_result = campos_potenciales(intensity, light_readings,&movements,max_advance,max_turn_angle,lidar_readings,params.laser_num_sensors,params.laser_value,params.laser_origin,params.laser_range);
                    }

                    if(cont_similar>=30 && cont_similar<40){
                        sm_avoid_obstacles_m(q_inputs,&movements,&est_sig, params.robot_max_advance * 0.8 ,params.robot_turn_angle * 0.5 ,12);
                        cont_similar++;
                    }

                    if(cont_similar==40){
                        cont_similar=0;
                        est_sig=0;
                    }


                    /*********************************************************************/

                    if(flg_result == 1)
                    {
                        if(flg_finish == 1) stop();
                        else
                        {
                            if(steps[i].node != -1)
                            {
                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x,final_y);
                                printf("New Light %d: x = %f  y = %f \n",i,final_x,final_y);
                                flg_finish=1;
                            }
                        }
                    }
                }

                break;

                case 13:
                //bug 1
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                    qx0 = params.robot_x;
                    qy0 = params.robot_y;
                    qx = 0;
                    qy = 0;
                    max_light_intensity = 0;
                    circledOnce = 0;
                    stepCounter = 0;
                }
                flg_result=sm_bug1(&stepCounter, params.robot_x, params.robot_y, &qx0, &qy0, &max_light_intensity, &circledOnce, lidar_readings, params.laser_num_sensors, params.laser_value, intensity, light_readings, q_light,q_inputs,&movements,&est_sig, params.robot_max_advance ,params.robot_turn_angle);

                if(flg_result == 1) stop();
                
                break;

                case 14:
               //bug 2
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                    qx0 = params.robot_x;
                    qy0 = params.robot_y;
                    qx = 0;
                    qy = 0;
                    qxdes = params.light_x;
                    qydes = params.light_y;
                    max_light_intensity = 0;
                    circledOnce = 0;
                    stepCounter = 0;
                    b = qxdes - qx0;
                    a = qydes - qy0;
                    c = a*qx0 + b*qy0;
                }
                flg_result=sm_bug2(a, b, c, &stepCounter, params.robot_x, params.robot_y, &qx0, &qy0, &max_light_intensity, &circledOnce, lidar_readings, params.laser_num_sensors, params.laser_value, intensity, light_readings, q_light,q_inputs,&movements,&est_sig, params.robot_max_advance ,params.robot_turn_angle);

                if(flg_result == 1) stop();
                break;


                default:
                    printf(" ******* SELECTION NO DEFINED *******\n");
                    movements.twist = 3.1416/4;
                    movements.advance = .03;
                break;
            }

            ros::spinOnce();
            printf("\n\n             MOTION PLANNER \n________________________________\n");
            printf("Light: x = %f  y = %f \n",params.light_x,params.light_y);
            printf("Robot: x = %f  y = %f \n",params.robot_x,params.robot_y);
            printf("Step: %d \n",cta_steps++);
            printf("Movement: twist: %f advance: %f \n" ,movements.twist ,movements.advance );

            flg_noise = params.noise;

            move_robot(movements.twist,movements.advance,lidar_readings);
            ros::spinOnce();
            new_simulation = 0;
            r.sleep();
        }
        ros::spinOnce();
        r.sleep();
    }
}
