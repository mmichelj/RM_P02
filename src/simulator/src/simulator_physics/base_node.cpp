#include "ros/ros.h"
#include <ros/package.h>
#include "simulator/Parameters.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_robot_step.h"
#include "simulator/simulator_parameters.h"
#include "simulator/simulator_base.h"
#include <string.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define MAX_NUM_POLYGONS 100
#define NUM_MAX_VERTEX 10
#define STRSIZ 300
#define SIZE_LINE 10000


typedef struct Vertex_ {
        float x;
        float y;
} Vertex;

typedef struct Polygon_ {
        char    name[ STRSIZ ];
        char    type[ STRSIZ ];
        int     num_vertex;
        Vertex  vertex[NUM_MAX_VERTEX];
        Vertex  min,max;
} Polygon;

Polygon polygons_wrl[100];
int num_polygons_wrl = 0;
parameters params;
char actual_world[50];
float dimensions_room_x,dimensions_room_y;

// it reads the file that conteins the environment description
int ReadPolygons(char *file,Polygon *polygons){

	FILE *fp;
	char data[ STRSIZ ];
	int i;
	int num_poly = 0;
	int flg = 0;
	float tmp;
	

	fp = fopen(file,"r"); 
	 
	if( fp == NULL )
	{
		sprintf(data, "File %s does not exist\n", file);
		printf("File %s does not exist\n", file);
		return(0);
	}
	//printf("World environment %s \n",file);

	while( fscanf(fp, "%s" ,data) != EOF)
	{
		if( strcmp(";(", data ) == 0 )
		{
			flg = 1;
			while(flg)
			{
				fscanf(fp, "%s", data);
				sscanf(data, "%f", &tmp);
				if(strcmp(")", data) == 0) flg = 0;
			}
		}
		else if((strcmp("polygon", data ) == 0) && ( flg == 0 ) )
		{
			fscanf(fp, "%s", data);
			strcpy(polygons[num_poly].type, data);
			fscanf(fp,"%s", data);
			strcpy(polygons[num_poly].name, data);
			i = 0;
			flg = 1;

			polygons[num_poly].max.x = 0;
			polygons[num_poly].max.y = 0;
			polygons[num_poly].min.x = 9999;
			polygons[num_poly].min.y = 9999;

			while(flg)
			{
				fscanf(fp,"%s",data);	
				if(strcmp(")",data) == 0) 
				{
					polygons[num_poly].num_vertex = i - 1;
					polygons[num_poly].vertex[i].x = polygons[num_poly].vertex[0].x; // to calculate intersecction range
					polygons[num_poly].vertex[i].y = polygons[num_poly].vertex[0].y; // the first vertex its repeated on the last
					num_poly++;
					flg = 0;
				}
				else
				{
					sscanf(data, "%f", &tmp);
					polygons[num_poly].vertex[i].x = tmp;
					fscanf(fp, "%s", data);
					sscanf(data, "%f", &tmp);
					polygons[num_poly].vertex[i].y = tmp;
					
					if(polygons[num_poly].vertex[i].x > polygons[num_poly].max.x)  polygons[num_poly].max.x = polygons[num_poly].vertex[i].x;
					if(polygons[num_poly].vertex[i].y > polygons[num_poly].max.y)  polygons[num_poly].max.y = polygons[num_poly].vertex[i].y;
					if(polygons[num_poly].vertex[i].x < polygons[num_poly].min.x)  polygons[num_poly].min.x = polygons[num_poly].vertex[i].x;
					if(polygons[num_poly].vertex[i].y < polygons[num_poly].min.y)  polygons[num_poly].min.y = polygons[num_poly].vertex[i].y;
	
					//printf("polygon vertex %d x %f y %f\n",i,polygons[num_poly].vertex[i].x,polygons[num_poly].vertex[i].y);
					i++;
				}
			}
		}
		else if(strcmp("dimensions", data) == 0  && (flg == 0) )
		{
			fscanf(fp, "%s", data);
			fscanf(fp, "%s", data);
			sscanf(data, "%f", &dimensions_room_x);
			fscanf(fp, "%s", data);
			sscanf(data, "%f", &dimensions_room_y);
			//printf("dimensions x %f y %f\n",dimensions_room_x,dimensions_room_y);
		}
	}
	fclose(fp);
	return num_poly;
}

void read_environment(char *file, int debug)
{

 	int i;                                                                            
	int j;
	/* it reads the polygons */
	strcpy(polygons_wrl[0].name, "NULL");
	if(debug == 1) printf("\nEnvironment file: %s\n", file);
	num_polygons_wrl = ReadPolygons(file, polygons_wrl);
	
	if(num_polygons_wrl == 0)
		printf("File doesnt exist %s \n",file);
	else  
		printf("Load: %s \n",file);                                                                                                                                                     
	// it prints the polygons
	if(debug == 1)
	for(i = 0; i < num_polygons_wrl; i++)
	{
		printf("\npolygon[%d].name=%s\n",i,polygons_wrl[i].name);
		printf("polygon[%d].type=%s\n",i,polygons_wrl[i].type);
		printf("Num vertex  polygon[%d].num_vertex=%d\n",i,polygons_wrl[i].num_vertex);
	    printf("max x,y = (%f, %f)  min x,y = (%f, %f) \n", polygons_wrl[i].max.x, polygons_wrl[i].max.y, polygons_wrl[i].min.x, polygons_wrl[i].min.y);
	    //printf("self.w.create_rectangle(%f* self.canvasX/2, (self.canvasY-( %f* self.canvasY )/2) ,  (%f* self.canvasX)/2, (self.canvasY-(%f* self.canvasX)/2), outline='#000000', width=1)\n", polygons_wrl[i].max.x, polygons_wrl[i].max.y, polygons_wrl[i].min.x, polygons_wrl[i].min.y);
		for(j = 0; j <= polygons_wrl[i].num_vertex+1 ; j++)
		{
			printf("polygon[%d].vertex[%d] x=%f y=%f\n", i, j, polygons_wrl[i].vertex[j].x, polygons_wrl[i].vertex[j].y);
			//printf("polygon[%d].line[%d] m=%f b=%f\n", i, j, polygons_wrl[i].line[j].m, polygons_wrl[i].line[j].b);
		}
	}
}

float pDistance(float x,float y,float x1,float y1,float x2,float y2) {

  float A = x - x1;
  float B = y - y1;
  float C = x2 - x1;
  float D = y2 - y1;

  float dot = A * C + B * D;
  float len_sq = C * C + D * D;
  float param = -1;
  float dx,dy;
  float  xx, yy;

  if (len_sq != 0) //in case of 0 length line
      param = dot / len_sq;

  if (param < 0) {
    xx = x1;
    yy = y1;
  }
  else if (param > 1) {
    xx = x2;
    yy = y2;
  }
  else {
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

   dx = x - xx;
   dy = y - yy;
   float aux =(dx * dx + dy * dy );

  return sqrt( aux );
}


int sat(float robot_x, float robot_y, float robot_r)
{
	int i,j;
	Vertex r_max;
	Vertex r_min;

	r_max.x = robot_x + robot_r; r_max.y = robot_y + robot_r;
	r_min.x = robot_x - robot_r; r_min.y = robot_y - robot_r;

	 //printf("self.w.create_rectangle(%f* self.canvasX, (self.canvasY-( %f* self.canvasY )) ,  (%f* self.canvasX), (self.canvasY-(%f* self.canvasX)), outline='#000000', width=1)\n", r_max.x, r_max.y, r_min.x, r_min.y);
		
	for(i = 0; i < num_polygons_wrl; i++)
		if( (r_min.x < polygons_wrl[i].max.x && polygons_wrl[i].max.x <   r_max.x) || ( r_min.x < polygons_wrl[i].min.x && polygons_wrl[i].min.x < r_max.x)  || ( polygons_wrl[i].min.x < r_min.x && r_max.x < polygons_wrl[i].max.x )  )
			if( (r_min.y < polygons_wrl[i].max.y && polygons_wrl[i].max.y < r_max.y) || ( r_min.y < polygons_wrl[i].min.y && polygons_wrl[i].min.y < r_max.y) || ( polygons_wrl[i].min.y < r_min.y && r_max.y < polygons_wrl[i].max.y )   )
				for(int j = 0; j <= polygons_wrl[i].num_vertex; j++)
		 			{
		 				//printf("Distancia al polig %f\n",pDistance(robot_x, robot_y, polygons_wrl[i].vertex[j].x, polygons_wrl[i].vertex[j].y, polygons_wrl[i].vertex[j + 1].x, polygons_wrl[i].vertex[j + 1].y));
		 				if( pDistance(robot_x, robot_y, polygons_wrl[i].vertex[j].x, polygons_wrl[i].vertex[j].y, polygons_wrl[i].vertex[j + 1].x, polygons_wrl[i].vertex[j + 1].y) <= robot_r ) 
						{	
						    return 1;
		 				}
		 				//else{ printf("Rx %f Ry %f v1x %f  V1y %f v2x %f v2y %f  \n",robot_x, robot_y, polygons_wrl[i].vertex[j].x, polygons_wrl[i].vertex[j].y, polygons_wrl[i].vertex[j + 1].x, polygons_wrl[i].vertex[j + 1].y) ;  }
		 			}	
	return 0;
}
	
bool check_path(simulator::simulator_base::Request  &req ,simulator::simulator_base::Response &res)
{
	float x1=req.x1;
	float y1=req.y1;
	float m = tan(req.theta);
	float x2,x22,y2,y22;
	float distance;
	char path[50];

	if (req.distance == 0)
		{res.distance = 0; return true;}



	if(m > 1 || m < -1 )
	{	
		y22 = req.distance * sin(req.theta) + y1;
		x2 = req.distance * cos(req.theta) + x1;
		y2 = 0;
		//printf("YY\n");
		if(y22 > y1)
		{	//printf("AAAA\n");
			for(y2 = y1; y2 <= y22; y2+=.005)
			{
				//y2 -y1= m ( x2 - x1)
				x2 =  (y2 - y1) / m + x1 ;
				
				if(sat(x2, y2, params.robot_radio))
				{//printf("y2:%f y1:%f \n",y2,y1);	
				break;}
			}
			if(x2 != x1)
			//printf("y1:%f x1:%f y2 %f x2 %f\n",y1,x1,y2,x2 );
			{	
				y2-=.005;
				x2 =  (y2 - y1) / m + x1 ;	
			}
			//printf("y1:%f x1:%f y2 %f x2 %f\n",y1,x1,y2,x2 );
			
		}
		else
		{
			//printf("BBB\n");
			for(y2 = y1; y2 >= y22; y2-=.005)
			{
				//y2 -y1= m ( x2 - x1)
				x2 =  (y2 - y1) / m + x1 ;
				if(sat(x2, y2, params.robot_radio))
				{//printf("Fuera\n");	
				break;}
			}
			if(x2 != x1)
			{
				y2+=.005;
				x2 =  (y2 - y1) / m + x1 ;
			}
		}

	}
	else
	{
		x22 = req.distance * cos(req.theta) + x1;
		y2 = req.distance * sin(req.theta) + y1;
		x2 = 0;

		if(x22-x1 >= 0)
		{
			//printf("CCC\n");
			for(x2 = x1; x2 <= x22; x2+=.005)
			{   
				y2 = m * (x2 - x1) + y1;
				//printf(" x: %f y: %f\n",x2*600,y2*600 );
				if(sat(x2, y2, params.robot_radio))
				{//printf("Fuera\n");
				break;}
			}
			if(x2 != x1)
			{
				x2-=.005;
				y2 = m * (x2 - x1) + y1;
			}
		}
		else
		{
			//printf("DDD\n");
			for(x2 = x1; x2 >= x22; x2-=.005)
			{
				y2 = m * (x2 - x1) + y1;
				if(sat(x2, y2, params.robot_radio))
					break;
			}
			if(x2 != x1)
			{
				x2+=.005;
				y2 = m * (x2 - x1) + y1;
			}
		}
	}
			
  	//printf("y1:%f x1:%f y2 %f x2 %f\n",y1,x1,y2,x2 );
    distance = sqrt( pow( x1-x2  ,2) + pow(y1-y2 ,2)  );
    if (req.distance < 0)
    	distance=-distance;
    res.distance = distance/dimensions_room_x;
   // printf("%f\n",res.distance );
    //printf("distance %f \n",distance);
   return true;
}


void paramsCallback(const simulator::Parameters::ConstPtr& paramss)
{
  std::string paths = ros::package::getPath("simulator");
  char path[100];


  params.robot_x             = paramss->robot_x   ;
  params.robot_y             = paramss->robot_y   ;
  params.robot_theta         = paramss->robot_theta   ;    
  params.robot_radio         = paramss->robot_radio   ;    
  params.robot_max_advance   = paramss->robot_max_advance   ;          
  params.robot_turn_angle    = paramss->robot_turn_angle   ;         
  params.laser_num_sensors   = paramss->laser_num_sensors   ;          
  params.laser_origin        = paramss->laser_origin         ;     
  params.laser_range         = paramss->laser_range   ;    
  params.laser_value         = paramss->laser_value   ;    
  strcpy(params.world_name ,paramss -> world_name.c_str());       
  params.noise               = paramss->noise   ;   
  params.run                 = paramss->run   ; 
  params.light_x             = paramss->light_x;
  params.light_y             = paramss->light_y;
  params.behavior            = paramss->behavior; 

    if(  strcmp( paramss->world_name.c_str(),actual_world) ) 
	{
		strcpy(path,paths.c_str());
		strcat(path,"/src/data/");
		strcat(path,paramss->world_name.c_str());
		strcat(path,"/");
		strcat(path,paramss->world_name.c_str());
		strcat(path,".wrl");
		read_environment(path,0);
		strcat(actual_world,paramss->world_name.c_str());
		strcpy(actual_world,paramss->world_name.c_str());
	}

}

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "simulator_base_node");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("simulator_base", check_path);
	ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);
		
	ros::spin();
	return 0;
}