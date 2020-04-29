#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/package.h>

typedef struct conectionba_
{
   int node;
   float cost;
}  conectionba;

typedef struct nodoba_
{
   char flag;
   int num_node;
   float x;
   float y;
   int num_conections;
	 conectionba conections[100];
   int parent;
   float acumulado;
}nodoba;

nodoba nodesba[250];
int num_nodesba = 0;

int read_nodesba(char *file)
{
   FILE *fp;
   char data[ 100 ];
   int i=0;
   int flg = 0;
   float tmp;
   float dimensions_room_x,dimensions_room_y;
   int node_index,node_connection,cost;

   fp = fopen(file,"r");

   if( fp == NULL )
   {
      sprintf(data, "File %s does not exists\n", file);
      printf("File %s does not exists\n", file);
      return(0);
   }

   while( fscanf(fp, "%s" ,data) != EOF)
   {
      if( strcmp(";(", data ) == 0 )
      {
         flg = 1;
         while(flg)
         {
            if(  0 < fscanf(fp, "%s", data));
            sscanf(data, "%f", &tmp);
            if(strcmp(")", data) == 0) flg = 0;
         }
      }
      else if((strcmp("node", data ) == 0) && ( flg == 0 ) )
      {
         if(  0 < fscanf(fp, "%s", data));
         nodesba[i].num_node = atoi(data);
         if(  0 < fscanf(fp, "%s", data));
         nodesba[i].x = atof(data);
         if(  0 < fscanf(fp, "%s", data));
         nodesba[i++].y = atof(data);
      }
      else if((strcmp("connection", data ) == 0) && ( flg == 0 ) )
      {
         if(  0 < fscanf(fp, "%s", data));
         node_index = atoi(data);

         if(  0 < fscanf(fp, "%s", data));
         node_connection = atoi(data);

         nodesba[node_index].conections[nodesba[node_index].num_conections].node = node_connection;

         if(  0 < fscanf(fp, "%s", data));
         nodesba[node_index].conections[nodesba[node_index].num_conections].cost = atof(data);
         nodesba[node_index].num_conections++;
      }
   }
   fclose(fp);
   return i;
}

void bfs_algorithm(int D ,int L)//start,goal
{
	int node_actual = D;
  int flagLoop = 1;

  printf("\n\n   Inicio %d\n",D);
  printf("   Fin %d\n",L);
  printf("   Numero de conexiones %d\n\n",nodesba[node_actual].num_conections);

  int arreglopadres[500];
  for(int i= 0; i<500; i++){
    arreglopadres[i]=-1;
  }
  int arreglohijos[500];
  for(int i=0; i<500; i++){
    arreglohijos[i]=-1;
  }
  int aux[500];
  for(int i=0; i<500; i++){
    aux[i]=-1;
  }
  arreglopadres[0] = node_actual;
  int j=0;
  int auxiliar = 0;
//Posible inicio de ciclo
  //for(int p = 0; p < 3; p++)
  while (flagLoop == 1)
  {
    int tamanio=0;
    for(int i = 0; i<500; i++){
      if(arreglopadres[i] != -1)
      tamanio=tamanio+1;
    }
    int hijostotales = 0;

     for(int i = 0; i < tamanio; i++)
    {
      node_actual=arreglopadres[i];
      hijostotales = hijostotales + nodesba[node_actual].num_conections;
    }
    // int arreglohijos[hijostotales];
    printf("Tamanio de padres %d\n",tamanio);
    printf("Hijos totales del arreglo padres %d\n",hijostotales);

    j=0;
    while(j < hijostotales)
    {
      for(int i = 0; i < tamanio; i++)
      {
        node_actual=arreglopadres[i];
        printf("Nodo actual: %d\n", node_actual);
        for(int k = 0; k < nodesba[node_actual].num_conections;k++)
        {
          if(nodesba[node_actual].conections[k].node != nodesba[node_actual].parent && nodesba[nodesba[node_actual].conections[k].node].parent == -1){
            arreglohijos[k+j]=nodesba[node_actual].conections[k].node;
            nodesba[nodesba[node_actual].conections[k].node].parent = nodesba[node_actual].num_node;
          }
          else
            arreglohijos[k+j]=-1;
        }
        j=j+nodesba[node_actual].num_conections;
        printf("J: %d\n",j);
        printf("Numero de conexion por nodo %d\n", nodesba[node_actual].num_conections);
      }
  	}

    printf("\n\n");
    printf("Conexiones\n");
    for( int i = 0; i < hijostotales; i++)
    {
      if(arreglohijos[j] != -1)
      printf("%d ",arreglohijos[i]);
  	}
    printf("\n\n");
    //cambia el arreglo padres por hijos
    auxiliar = 0;
    for(int i = 0; i<500; i++){
      if(i<hijostotales)
      {
        if(arreglohijos[i] != -1){
          aux[i-auxiliar]=arreglohijos[i];
        }
        else{
          auxiliar = auxiliar + 1;
        }
      }
      else
      arreglopadres[i]=-1;
    }

    for(int i = 0; i<500; i++){
      if(i<hijostotales){
        if(aux[i] != L){
          arreglopadres[i]=aux[i];
        }
        else
        {
          printf("\n\n\nMETA ENCONTRADA\n");
          printf("%d\n", aux[i]);
          printf("Padre de la meta %d\n\n\n", nodesba[aux[i]].parent);
          flagLoop = 0;
        }
      }
      else
      arreglopadres[i]=-1;
    }
    for(int i= 0; i<500; i++){
      aux[i]=-1;
    }

    printf("Arreglo padres\n");
    for(int i = 0; i<500; i++){
      printf("%d ",arreglopadres[i]);
    }
    printf("\n");

    for(int i= 0; i<500; i++){
      arreglohijos[i]=-1;
    }
    printf("Arreglo hijos\n");
    for(int i = 0; i<500; i++){
      printf("%d ",arreglohijos[i]);
    }
    printf("\n");
  }
}

int bfs(float rx ,float ry ,float lx ,float ly, char *world_name,step *steps)
{
   //char archivo[]="../data/obstacles/obstacles.top";
   char archivo[150];
   int i;
   int start = 0;
   int goal = 0;
   int padre;
   std::string paths = ros::package::getPath("simulator");
   strcpy(archivo,paths.c_str());
   strcat(archivo,"/src/data/");
   strcat(archivo,world_name);
   strcat(archivo,"/");
   strcat(archivo,world_name);
   strcat(archivo,".top");

   for(i = 0; i < 250; i++)
   {
      nodesba[i].flag='N';
      nodesba[i].num_conections = 0;
      nodesba[i].parent = -1;
      nodesba[i].acumulado = 0;
   }

   num_nodesba=read_nodesba(archivo); // Se lee el arcivo .top

   for(i = 1; i < num_nodesba; i++)
   {
   		if( sqrt(pow( nodesba[i].x - rx ,2) + pow( nodesba[i].y - ry ,2)) < sqrt( pow( nodesba[start].x - rx ,2) + pow( nodesba[start].y - ry ,2)) )
   			start = i;

   		if( sqrt(pow( nodesba[i].x - lx ,2) + pow( nodesba[i].y - ly ,2)) < sqrt(pow( nodesba[goal].x - lx ,2) + pow( nodesba[goal].y - ly ,2) ) )
   			goal = i;
   }

   bfs_algorithm(goal,start);
   padre = start;
   i = 0;

   while( padre != -1)
   {
     steps[i].node = nodesba[padre].num_node;
     steps[i].x = nodesba[padre].x;
     steps[i].y = nodesba[padre].y;
       i++;
     padre = nodesba[padre].parent;
   }
	return 0;
}
