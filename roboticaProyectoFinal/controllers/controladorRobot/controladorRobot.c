#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define GRID_SIZE 80  
#define CELL_SIZE 0.5
#define MAX_PATH_LEN 100
#define MAX_OPEN_NODES 1000
#define SPEED 10.0


typedef struct {
  int x, y;
} Point;

typedef struct {
  int x, y;
  int g, h, f;
  int parent_index;
} Node;

Point world_to_grid(double x, double y) {
  Point p;
  p.x = (int)((x + (GRID_SIZE * CELL_SIZE) / 2.0) / CELL_SIZE);
  p.y = (int)((y + (GRID_SIZE * CELL_SIZE) / 2.0) / CELL_SIZE);
  return p;
}

int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int plan_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int max_path_len) {
  Node open_list[MAX_OPEN_NODES];
  int open_count = 0;

  Node closed_list[GRID_SIZE * GRID_SIZE];
  int closed_count = 0;

  Node start_node = {start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), 0, -1};
  start_node.f = start_node.g + start_node.h;
  open_list[open_count++] = start_node;

  while (open_count > 0) {
    int best_index = 0;
    for (int i = 1; i < open_count; i++) {
      if (open_list[i].f < open_list[best_index].f)
        best_index = i;
    }

    Node current = open_list[best_index];
    for (int i = best_index; i < open_count - 1; i++)
      open_list[i] = open_list[i + 1];
    open_count--;
    closed_list[closed_count++] = current;

    if (current.x == goal.x && current.y == goal.y) {
      int length = 0;
      Node n = current;
      while (n.parent_index != -1 && length < max_path_len) {
        path[length++] = (Point){n.x, n.y};
        n = closed_list[n.parent_index];
      }
      path[length++] = (Point){start.x, start.y};
      for (int i = 0; i < length / 2; i++) {
        Point temp = path[i];
        path[i] = path[length - i - 1];
        path[length - i - 1] = temp;
      }
      return length;
    }

    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};
    for (int d = 0; d < 4; d++) {
      int nx = current.x + dx[d];
      int ny = current.y + dy[d];
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
        continue;
      if (grid[nx][ny] == 1)
        continue;

      int in_closed = 0;
      for (int i = 0; i < closed_count; i++) {
        if (closed_list[i].x == nx && closed_list[i].y == ny) {
          in_closed = 1;
          break;
        }
      }
      if (in_closed) continue;

      int g = current.g + 1;
      int h = heuristic(nx, ny, goal.x, goal.y);
      int f = g + h;

      int in_open = -1;
      for (int i = 0; i < open_count; i++) {
        if (open_list[i].x == nx && open_list[i].y == ny) {
          in_open = i;
          break;
        }
      }

      if (in_open != -1) {
        if (f < open_list[in_open].f) {
          open_list[in_open].g = g;
          open_list[in_open].h = h;
          open_list[in_open].f = f;
          open_list[in_open].parent_index = closed_count - 1;
        }
      } else if (open_count < MAX_OPEN_NODES) {
        Node neighbor = {nx, ny, g, h, f, closed_count - 1};
        open_list[open_count++] = neighbor;
      }
    }
  }
  return 0;
}
 
int main() {
  wb_robot_init();
  double left_speed = 1.0;
  double right_speed = 1.0;
  int i;

  //motores
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  //sensores 
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  //lidar
  WbDeviceTag lidar=wb_robot_get_device("lidar");
  wb_lidar_enable(lidar,TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  
  //GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  //ruta grilla
  int grid[GRID_SIZE][GRID_SIZE] = {0};
  Point path[100];
  int path_length = 0;
  
  while (wb_robot_step(TIME_STEP) != -1) {//<---------------------------------------------------------------------inicio del ciclo while
  
   int current_waypoint = 1;  // Índice del waypoint actual (1 = primer punto después de la posición actual)
   double WAYPOINT_THRESHOLD = 0.3;  // Distancia para considerar waypoint alcanzado
   
   //Flags
   bool ds_detect_near=false;
   bool lidar_detect_near=false;
   
   left_speed = 5.0;
   right_speed = 5.0;
   

   double ds_values[2];
   for (i = 0; i < 2; i++){
     ds_values[i] = wb_distance_sensor_get_value(ds[i]);
     //printf("Sensor %d: %.2f\n", i, ds_values[i]);
     if (ds_values[i] < 100.0) ds_detect_near=true;
   }
    
    //GPS
    const double *pose = wb_gps_get_values(gps);
    float robot_x = pose[0];
    float robot_y = pose[2]; 
  
    
    //lidar
    const float *ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    
    for (int i = 0; i < resolution; i++) {
      double angle = -fov / 2 + i * (fov / resolution);
      double dist = ranges[i];
      
      if(isinf(dist)) continue; 
      if (dist < 1.0) {
        float obs_x = robot_x + dist * cos(angle);
        float obs_y = robot_y + dist * sin(angle);

        int cell_x = (int)((obs_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        int cell_y = (int)((obs_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);

        if (cell_x >= 0 && cell_x < GRID_SIZE && cell_y >= 0 && cell_y < GRID_SIZE)
          grid[cell_x][cell_y] = 1;  // Marcado como ocupado
      }
      
      if(dist < 0.15)
        lidar_detect_near=true; //obstaculo muy cerca girar
    
      } 
      printf("posicion del robot actual: %f, %f\n", robot_x, robot_y);
      //plan path
     //Planificación 
      Point start = world_to_grid(robot_x, robot_y);
      Point goal = world_to_grid( -4.649370, 0.06975);
      printf("punto actualen el mapa %i, %i\n", start.x, start.y);
      printf("punto goal %i, %i\n", goal.x, goal.y);
      if(start.x == goal.x && start.y == goal.y){
        while(true){
          left_speed = 0;
          right_speed = 0;
        }
      }
            
      path_length = plan_path(grid, start, goal, path, 100);
      printf("posibles pasos necesarios: %d\n", path_length);

      if (lidar_detect_near || ds_detect_near) {
          printf("Comportamiento reactivo de evasión\n");
          left_speed = 5.0;
          right_speed = -5.0;
      } else if (path_length > 1) {
          if (current_waypoint >= path_length) {
              left_speed = 0;
              right_speed = 0;
              printf("¡Destino alcanzado!\n");
          } else {
              // Convertir waypoint a coordenadas mundiales
              printf("Entrar manejo de waypoint\n");
              Point next_wp = path[current_waypoint];
              double target_x = (next_wp.x * CELL_SIZE) - (GRID_SIZE * CELL_SIZE / 2.0);
              double target_y = (next_wp.y * CELL_SIZE) - (GRID_SIZE * CELL_SIZE / 2.0);
              
              // Calcular dirección hacia el waypoint
              double dx = target_x - robot_x;
              double dy = target_y - robot_y;
              double distance_to_wp = sqrt(dx*dx + dy*dy);
              double target_angle = atan2(dy, dx);
              
              // Control direccional básico
              double angle_diff = target_angle; // Simplificado (en implementación real necesitarías la orientación actual del robot)
              
              if (distance_to_wp < WAYPOINT_THRESHOLD) {
                  // Avanzar al siguiente waypoint
                  current_waypoint++;
                  printf("Waypoint %d/%d alcanzado\n", current_waypoint, path_length);
              } else if (angle_diff > 0.1) {
                  printf("Girar a la derecha\n");
                  left_speed = SPEED;
                  right_speed = SPEED * 0.3;
              } else if (angle_diff < -0.1) {
                  printf("Girar a la izquierda\n");
                  left_speed = SPEED * 0.3;
                  right_speed = SPEED;
              } else {
                  printf("Avanzar recto\n");
                  left_speed = SPEED;
                  right_speed = SPEED;
              }
          }
      }
    
      
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
     
    
    fflush(stdout); 
    
  };

  
  wb_robot_cleanup();

  return 0;
}