
/*
  Data is sent in format:
  
  {"timestamp": 1561043647.7598028, 
            "vehicle.imu": {"timestamp": 1561043647.7431362, 
                    "angular_velocity": [-8.910427823138889e-06, 1.6135254554683343e-06, 0.0005768465343862772], 
                    "linear_acceleration": [-0.06396577507257462, 0.22235631942749023, 9.807276725769043], 
                    "magnetic_field": [23662.052734375, 2878.55859375, -53016.55859375]}, 
                    "vehicle.gps": {"timestamp": 1561043647.7431362, "x": -0.0027823783457279205, "y": -0.026340210810303688, "z": 0.159392312169075}, 
                    "vehicle.velocity": {"timestamp": 1561043647.7431362, "linear_velocity": [-6.0340113122947514e-05, -2.264878639834933e-05, 9.702569059300004e-07], 
                    "angular_velocity": [-8.910427823138889e-06, 1.6135254554683343e-06, 0.0005768465343862772], 
                    "world_linear_velocity": [-5.9287678595865145e-05, -2.5280191039200872e-05, 8.493661880493164e-07]}, 
                    "vehicle.pose": {"timestamp": 1561043647.7431362, "x": -0.0027823783457279205, "y": -0.026340210810303688, "z": 0.159392312169075, "yaw": 0.04371734336018562, "pitch": 0.0065115075558424, "roll": 0.022675735875964165}}
*/


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include "ardupilot_SITL_QUAD.h"
#include "sockets.h"
#include "sensors.h"

//_________________________________________


#include <stdio.h>
#include <sys/time.h>
#include <webots/supervisor.h>

#define M_PI  3.14159265358979323846
#define M_PI2 6.28318530718


//_________________________________________



bool socket_init() {
#ifdef _WIN32 /* initialize the socket API */
  WSADATA info;
  if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
    fprintf(stderr, "Cannot initialize Winsock.\n");
    return false;
  }
#endif
  return true;
}

bool socket_set_non_blocking(int fd) {
  if (fd < 0)
    return false;
#ifdef _WIN32
  unsigned long mode = 1;
  return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? true : false;
#else
  int flags = fcntl(fd, F_GETFL, 0) | O_NONBLOCK;
  return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
#endif
}

int socket_accept(int server_fd) {
  int cfd;
  struct sockaddr_in client;
  struct hostent *client_info;
#ifndef _WIN32
  socklen_t asize;
#else
  int asize;
#endif
  asize = sizeof(struct sockaddr_in);
  cfd = accept(server_fd, (struct sockaddr *)&client, &asize);
  if (cfd == -1) {
#ifdef _WIN32
    int e = WSAGetLastError();
    if (e == WSAEWOULDBLOCK)
      return 0;
    fprintf(stderr, "Accept error: %d.\n", e);
#else
    if (errno == EWOULDBLOCK)
      return 0;
    fprintf(stderr, "Accept error: %d.\n", errno);
#endif
    return -1;
  }
  client_info = gethostbyname((char *)inet_ntoa(client.sin_addr));
  printf("Accepted connection from: %s.\n", client_info->h_name);
  return cfd;
}

bool socket_close(int fd) {
#ifdef _WIN32
  return (closesocket(fd) == 0) ? true : false;
#else
  return (close(fd) == 0) ? true : false;
#endif
}

bool socket_cleanup() {
#ifdef _WIN32
  return (WSACleanup() == 0) ? true : false;
#else
  return true;
#endif
}




/*
  Creates a socket and bind it to port.
 */
int create_socket_server(int port) {
  int sfd, rc;
  struct sockaddr_in address;
  if (!socket_init())
  {
    fprintf (stderr, "socket_init failed");
    return -1;
  }
  sfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sfd == -1) {
    fprintf(stderr, "Cannot create socket.\n");
    return -1;
  }
  int one0 = 1;
  const void *one1 = &one0;
  setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, one1, sizeof(one1));
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;
  rc = bind(sfd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    fprintf(stderr, "Cannot bind port %d.\n", port);
    socket_close(sfd);
    return -1;
  }
  if (listen(sfd, 1) == -1) {
    fprintf(stderr, "Cannot listen for connections.\n");
    socket_close(sfd);
    return -1;
  }
  
  printf ("socket initialized at port %d.\n", port);
  return sfd;
}




//_______________________________________________________________________________________________________________________________________________________
/*
https://discuss.ardupilot.org/t/copter-x-y-z-which-is-which/6823/2

Copy pasted what’s important:
NED Coordinate System:

The x axis is aligned with the vector to the north pole (tangent to meridians).
The y axis points to the east side (tangent to parallels)
The z axis points to the center of the earth
There is also Body Fixed Frame:
Body Fixed Frame (Attached to the aircraft)

The x axis points in forward (defined by geometry and not by movement) direction. (= roll axis)
The y axis points to the right (geometrically) (= pitch axis)
The z axis points downwards (geometrically) (= yaw axis)
In order to convert from Body Frame to NED what you need to call this function:

copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);




 */
/*
  returns: "yaw":_6.594794831471518e-05,"pitch":_-0.0005172680830582976,"roll":_0.022908752784132957}}
*/
void getInertia (const WbDeviceTag inertialUnit, char *buf)
{
  const double *inertial_directions = wb_inertial_unit_get_roll_pitch_yaw (inertialUnit);
  
  sprintf(buf,"\"roll\": %f,\"pitch\": %f,\"yaw\": %f",inertial_directions[0], inertial_directions[1], inertial_directions[2]);
  
  return ;

}

/*
  returns: "magnetic_field":_[23088.669921875,_3876.001220703125,_-53204.57421875]
*/
void getCompass (const WbDeviceTag compass, char *buf)
{
    const double *north3D = wb_compass_get_values(compass);
    
    sprintf(buf,"[%f, %f, %f]",north3D[0], north3D[2], north3D[1]);
    
    return ;

}


double old_north3D[3];
double lllinear_velocity[3];
double llspeed;
/*
  returns: "vehicle.gps":{"timestamp":_1563301031.055164,"x":_5.5127296946011484e-05,"y":_-0.0010968948481604457,"z":_0.037179552018642426}, 
*/
void getGPS (const WbDeviceTag gps, char *buf)
{

    const double *north3D = wb_gps_get_values(gps);
    llspeed = wb_gps_get_speed(gps);
    const double delta = (north3D[0] - old_north3D[0]);
    if (delta != 0.0)
    {
      lllinear_velocity[0] = (north3D[0] - old_north3D[0]) * timestep_scale;
      lllinear_velocity[1] = (north3D[1] - old_north3D[1]) * timestep_scale;
      lllinear_velocity[2] = (north3D[2] - old_north3D[2]) * timestep_scale;
    }
    old_north3D[0] = north3D[0];
    old_north3D[1] = north3D[1];
    old_north3D[2] = north3D[2];

    sprintf(buf,"\"x\": %f,\"y\": %f,\"z\": %f", north3D[0], north3D[2], north3D[1]);
    
    return ;
}

/*
 returns: "linear_acceleration": [0.005074390675872564, 0.22471477091312408, 9.80740737915039]
*/
void getAcc (const WbDeviceTag accelerometer, char *buf)
{
    //SHOULD BE CORRECT 
    const double *a = wb_accelerometer_get_values(accelerometer);
    
    sprintf(buf,"[%f, %f, %f]",a[0], a[2], a[1]);
    
    //sprintf(buf,"[0.0, 0.0, 0.0]");

    return ;
}


/*
  returns: "angular_velocity": [-1.0255117643964695e-07, -8.877226775894087e-08, 2.087078510015772e-09]
*/
void getGyro (const WbDeviceTag gyro, char *buf)
{

    const double *g = wb_gyro_get_values(gyro);
    
    sprintf(buf,"[%f, %f, %f]",g[0], g[2], g[1]);
    
    return ;
}


void getLinearVelocity (WbNodeRef nodeRef,  char * buf)
{
    if (linear_velocity != NULL)
    {
      sprintf (buf,"[%f, %f, %f]", lllinear_velocity[0], lllinear_velocity[2], lllinear_velocity[1]);
      //sprintf (pBug,"[%f, %f, %f]", lllinear_velocity[0], lllinear_velocity[2], lllinear_velocity[1]);
    }

    return ;
}

void getAllSensors (char *buf, WbDeviceTag gyro, WbDeviceTag accelerometer, WbDeviceTag compass, const WbDeviceTag gps, const WbDeviceTag inertial_unit)
{

/*
{"timestamp": 1563544049.2840538, 
    "vehicle.imu": {"timestamp": 1563544049.2673872, 
    "angular_velocity": [-2.0466000023589004e-06, 3.1428675129063777e-07, -6.141597896913709e-09],
    "linear_acceleration": [0.005077465437352657, 0.22471386194229126, 9.807389259338379], 
    "magnetic_field": [23088.71484375, 3875.498046875, -53204.48046875]}, 
    "vehicle.gps": {
        "timestamp": 1563544049.2673872, 
        "x": 4.985610576113686e-05, "y": -0.0010973707539960742, "z": 0.037179529666900635}, 
    "vehicle.velocity": {"timestamp": 1563544049.2673872, 
                        "linear_velocity": [-3.12359499377024e-10, -1.3824124955874595e-08, -6.033386625858839e-07],
                        "angular_velocity": [-2.0466000023589004e-06, 3.1428675129063777e-07, -6.141597896913709e-09], 
                        "world_linear_velocity": [0.0, 0.0, -6.034970851942489e-07]}, 
    "vehicle.pose": {"timestamp": 1563544049.2673872, 
                            "x": 4.985610576113686e-05, "y": -0.0010973707539960742, "z": 0.037179529666900635, "yaw": 8.899446402210742e-05, "pitch": -0.0005175824626348913, "roll": 0.022908702492713928}
                            }
*/
        

        static char compass_buf [150];
        static char acc_buf [150];
        static char gyro_buf [150];
        static char gps_buf [150];
        static char inertial_buf [150];
        static char linear_velocity_buf [150];

        char szTime[21];
        double time = wb_robot_get_time(); // current simulation time in [s]
        sprintf(szTime,"%lf", time);
        
        getGyro(gyro, gyro_buf);
        getAcc(accelerometer, acc_buf);
        getCompass(compass, compass_buf);
        getGPS(gps, gps_buf);
        getInertia (inertial_unit, inertial_buf);
        getLinearVelocity(self_node, linear_velocity_buf);

        sprintf (buf,"{\"ts\": %s,\"vehicle.imu\": {\"av\": %s,\"la\": %s,\"mf\": %s,\"vehicle.gps\":{%s},\"vehicle.velocity\":{\"wlv\": %s},\"vehicle.pose\":{%s,%s}}\r\n"
                      , szTime,                     gyro_buf,    acc_buf,   compass_buf,               gps_buf,                                  linear_velocity_buf,               gps_buf, inertial_buf );

}




//_______________________________________________________________________________________________________________________________________________________
#define MOTOR_NUM 4

static WbDeviceTag motors[MOTOR_NUM];

static WbDeviceTag gyro;
static WbDeviceTag accelerometer;
static WbDeviceTag compass;
static WbDeviceTag gps;
static WbDeviceTag camera;
static WbDeviceTag inertialUnit;
static WbDeviceTag emitter;


static double _linear_velocity[3] = {0.0,0.0,0.0};
static double v[MOTOR_NUM];
int port;
float dragFactor = VEHICLE_DRAG_FACTOR;

static int timestep;

#ifdef DEBUG_SENSORS
FILE *fptr;
#endif

#ifdef DEBUG_USE_KB
/*
// Code used tp simulae motors using keys to make sure that sensors directions and motor torques and thrusts are all correct.
// You can start this controller and use telnet instead of SITL to start the simulator.
Then you can use Keyboard to emulate motor input.
*/
static float factor = 1.0f;
static float offset = 0.0f;
void process_keyboard ()
{
  switch (wb_keyboard_get_key()) 
  {
    case 'Q':  // Q key -> up & left
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
      v[3] = 0.0;
      break;

    case 'Y':
      v[0] = v[0] + 0.01;
      v[2] = v[2] - 0.01;
      break;

    case 'H':
      v[0] = v[0] - 0.01;
      v[2] = v[2] + 0.01;
      break;

    case 'G':
      v[1] = v[1] + 0.01;
      v[3] = v[3] - 0.01;
      break;

    case 'J':
      v[1] = v[1] - 0.01;
      v[3] = v[3] + 0.01;
      break;

    case 'E':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v[i] += 0.00002;
      }
      break;

    case 'W':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v[i] += 0.0002;
      }
      break;

    case 'S':
      for (int i=0; i<MOTOR_NUM;++i)
      {
        v[i] -= 0.0002;
      }
      break;
  
    case 'A':
      v[1] = v[1] + 0.001;
      v[3] = v[3] + 0.001;
      v[0] = v[0] - 0.001;
      v[2] = v[2] - 0.001;
      break;

    case 'D':
      v[1] = v[1] - 0.001;
      v[3] = v[3] - 0.001;
      v[0] = v[0] + 0.001;
      v[2] = v[2] + 0.001;
      break;

    
  }

  for (int i=0; i< MOTOR_NUM; ++i)
  {
    
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], factor* v[i] + offset); 

    
  }
  
  printf ("Motors Internal %f %f %f %f\n", v[0],v[1],v[2],v[3]);
  
}
#endif


/*
// apply motor thrust.
*/
void update_controls()
{
  /*
      1 N = 101.97162129779 grams force
      Thrust = t1 * |omega| * omega - t2 * |omega| * V
      Where t1 and t2 are the constants specified in the thrustConstants field,
      omega is the motor angular velocity 
      and V is the component of the linear velocity of the center of thrust along the shaft axis.

      if Vehicle mass = 1 Kg. and we want omega = 1.0 to hover
      then (mass / 0.10197) / (4 motors) = t1

    LINEAR_THRUST
      we also want throttle to be linear with thrust so we use sqrt to calculate omega from input.
      Check this doc: https://docs.google.com/spreadsheets/d/1eR4Fb6cgaTb-BHUKJbhAXPzyX0ZLtUcEE3EY-wQYvM8/edit?usp=sharing
   */
#ifndef DEBUG_USE_KB
  //static float factor = 2.1f;
  //static float offset = 0.0f;
  
  
  //static float factor = 1.0f;
  static float offset = 0.0f;
  
  static float v[4];
  // pls check https://docs.google.com/spreadsheets/d/1eR4Fb6cgaTb-BHUKJbhAXPzyX0ZLtUcEE3EY-wQYvM8/edit?usp=sharing
  static float factorDyn[11] = {
            3.6f, // 0.0
            3.6f, // 0.1
            4.6f, // 0.2
            4.1f, // 0.3
            4.1f, // 0.4
            3.9f, // 0.5
            3.9f, // 0.6
            3.8f, // 0.7
            3.7f, // 0.8 
            3.6f, // 0.9 
            3.4f  // 1.0
          };
  //#define LINEAR_THRUST



#ifdef LINEAR_THRUST
  v[0] = sqrt(state.motors.w) * factor + offset;
  v[1] = sqrt(state.motors.x) * factor + offset;
  v[2] = sqrt(state.motors.y) * factor + offset;
  v[3] = sqrt(state.motors.z) * factor + offset;
#else  
  v[0] = (state.motors.w ) * factorDyn[(int)(10 * state.motors.w)] + offset;
  v[1] = (state.motors.x ) * factorDyn[(int)(10 * state.motors.x)]  + offset;
  v[2] = (state.motors.y ) * factorDyn[(int)(10 * state.motors.y)]  + offset;
  v[3] = (state.motors.z ) * factorDyn[(int)(10 * state.motors.z)]  + offset;
#endif //LINEAR_THRUST

  for ( int i=0; i<4; ++i)
  {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], v[i]); 
  }

  #ifdef DEBUG_MOTORS
  printf ("RAW    F:%f  R:%f  B:%f  L:%f\n", state.motors.w, state.motors.x, state.motors.y, state.motors.z);
  printf ("Factors    F:%f  \n", factorDyn[(int)(10 * state.motors.w)]);
  printf ("Motors F:%f  R:%f  B:%f  L:%f\n", v[0], v[1], v[2], v[3]);
  
  #endif //DEBUG_MOTORS

#endif //DEBUG_USE_KB

  #ifdef WIND_SIMULATION
  /*
    Drag: Fd = ½ ρ Cd A v²

    Fd is drag force in Newtons
    ρ is the density of air in kg/m³
    Cd is the drag coefficient
    A is the cross section of our quad in m³ in the direction of movement
    v is the velocity in m/s
  */
  
  wind_webots_axis.x =  state.wind.x - linear_velocity[0];
  wind_webots_axis.z = -state.wind.y - linear_velocity[2];   // "-state.wind.y" as angle 90 wind is from EAST.
  wind_webots_axis.y =  state.wind.z - linear_velocity[1];
  

  wind_webots_axis.x = dragFactor * wind_webots_axis.x * abs(wind_webots_axis.x);
  wind_webots_axis.z = dragFactor * wind_webots_axis.z * abs(wind_webots_axis.z);
  wind_webots_axis.y = dragFactor * wind_webots_axis.y * abs(wind_webots_axis.y);

  wb_emitter_send(emitter, &wind_webots_axis, sizeof(VECTOR4F));
  
  #ifdef DEBUG_WIND
  printf("wind sitl: %f %f %f %f\n",state.wind.w, state.wind.x, state.wind.y, state.wind.z);
  printf("wind ctrl: (dragFactor) %f %f %f %f %f\n",dragFactor, wind_webots_axis.w, wind_webots_axis.x, wind_webots_axis.y, wind_webots_axis.z);
  #endif

  #endif
}


// data example: [my_controller_SITL] {"engines": [0.000, 0.000, 0.000, 0.000]}
// the JSON parser is directly inspired by https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_Morse.cpp
bool parse_controls(const char *json)
{
    #ifdef DEBUG_INPUT_DATA
    printf("%s\n", json);
    #endif
    
    for (uint16_t i=0; i < ARRAY_SIZE(keytable); i++) {
        struct keytable *key;
        key = &keytable[i];
        // look for section header 
        const char *p = strstr(json, key->section);
        if (!p) {
            // we don't have this sensor
            continue;
        }
        p += strlen(key->section)+1;

        // find key inside section
        p = strstr(p, key->key);
        if (!p) {
            fprintf(stderr,"Failed to find key %s/%s DATA:%s\n", key->section, key->key, json);
            return false;
        }

        p += strlen(key->key)+3;
        
        switch (key->type) 
        {
          case DATA_FLOAT:
              *((float *)key->ptr) = atof(p);
              #ifdef DEBUG_INPUT_DATA
              printf("GOT  %s/%s\n", key->section, key->key);
              #endif
              break;

          case DATA_DOUBLE:
              *((double *)key->ptr) = atof(p);
              #ifdef DEBUG_INPUT_DATA
              printf("GOT  %s/%s\n", key->section, key->key);
              #endif
              break;

          case DATA_VECTOR4F: {
              VECTOR4F *v = (VECTOR4F *)key->ptr;
              if (sscanf(p, "[%f, %f, %f, %f]", &(v->w), &(v->x), &(v->y), &(v->z)) != 4) {
                  fprintf(stderr,"Failed to parse Vector3f for %s %s/%s\n",p,  key->section, key->key);
                  return false;
              }
              else
              {
                  #ifdef DEBUG_INPUT_DATA
                  printf("GOT  %s/%s\n[%f, %f, %f, %f]\n ", key->section, key->key,v->w,v->x,v->y,v->z);
                  #endif
              }
              
              break;
              }
        }
    }
    return true;
}

void run ()
{

    char send_buf[1000]; 
    char command_buffer[1000];
    fd_set rfds;
    
    // calculate initial sensor values.
    wb_robot_step(timestep);
    
    while (true) 
    {
        #ifdef DEBUG_USE_KB
        process_keyboard();
        #endif

        if (fd == 0) 
        {
          // if no socket wait till you get a socket
            fd = socket_accept(sfd);
            if (fd < 0)
              break;
        }
         
        
        // trigget ArduPilot to send motor data 
        getAllSensors ((char *)send_buf, gyro,accelerometer,compass,gps, inertialUnit);

        #ifdef DEBUG_SENSORS
        //printf("at %lf  %s\n",wb_robot_get_time(), send_buf);
        printf("at %lf  %s\n",wb_robot_get_time(), send_buf);
        if (strlen (pBug)> 5)
        {
        // fprintf(fptr, "%s\n",pBug);
        }
        #endif
         
        
        if (write(fd,send_buf,strlen(send_buf)) <= 0)
        {
          fprintf (stderr,"Send Data Error\n");
        }

        if (fd) 
        {
          FD_ZERO(&rfds);
          FD_SET(fd, &rfds);
          struct timeval tv;
          tv.tv_sec = 0.05;
          tv.tv_usec = 0;
          int number = select(fd + 1, &rfds, NULL, NULL, &tv);
          if (number != 0) 
          {
            // there is a valid connection
                int n = recv(fd, (char *)command_buffer, 1000, 0);

                if (n < 0) {
        #ifdef _WIN32
                  int e = WSAGetLastError();
                  if (e == WSAECONNABORTED)
                    fprintf(stderr, "Connection aborted.\n");
                  else if (e == WSAECONNRESET)
                    fprintf(stderr, "Connection reset.\n");
                  else
                    fprintf(stderr, "Error reading from socket: %d.\n", e);
        #else
                  if (errno)
                    fprintf(stderr, "Error reading from socket: %d.\n", errno);
        #endif
                  break;
                }
                if (n==0)
                {
                  break;
                }
                if (n > 0)
                {
                  command_buffer[n] = 0;
                  if (parse_controls (command_buffer))
                  {
                    update_controls();
                    //https://cyberbotics.com/doc/reference/robot#wb_robot_step
                    // this is used to force webots not to execute untill it receives feedback from simulator.
                    wb_robot_step(timestep);
                  }

                }
          }
          
        }
    }
    
    socket_cleanup();
}


bool initialize (int argc, char *argv[])
{
  
  fd_set rfds;
  #ifdef DEBUG_SENSORS
  fptr = fopen ("/tmp/log.txt","w");
  #endif
  port = 5599;  // default port
  for (int i = 0; i < argc; ++i)
  {
      if (strcmp (argv[i],"-p")==0)
      { // specify port for SITL.
        if (argc > i+1 )
        {
          port = atoi (argv[i+1]);
          printf("socket port %d\n",port);
        }
      }
      else if (strcmp (argv[i],"-df")==0)
      { // specify drag functor used to simulate air resistance.
        if (argc > i+1 )
        {
          dragFactor = atof (argv[i+1]);
          printf("drag Factor %f\n",dragFactor);
        }
        else
        {
          fprintf(stderr,"Missing drag factor value.\n");
          return false;
        }
        
      }
  }
    
    
  sfd = create_socket_server(port);
  
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  timestep = (int)wb_robot_get_basic_time_step();
  timestep_scale = timestep * 1000.0;
  printf("timestep_scale: %f \n", timestep_scale);
  
  
  // keybaard
  #ifdef DEBUG_USE_KB
  wb_keyboard_enable(timestep);
  #endif


  // inertialUnit
  inertialUnit = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(inertialUnit, timestep);

  // gyro
  gyro = wb_robot_get_device("gyro1");
  wb_gyro_enable(gyro, timestep);

  // accelerometer
  accelerometer = wb_robot_get_device("accelerometer1");
  wb_accelerometer_enable(accelerometer, timestep);
  
  // compass
  compass = wb_robot_get_device("compass1");
  wb_compass_enable(compass, timestep);

  // gps
  gps = wb_robot_get_device("gps1");
  wb_gps_enable(gps, timestep);

  // camera
  camera = wb_robot_get_device("camera1");
   wb_camera_enable(camera, CAMERA_FRAME_RATE_FACTOR * timestep);

  #ifdef WIND_SIMULATION
  // emitter
  emitter = wb_robot_get_device("emitter_plugin");
  #endif

  // names of motor should be the same as name of motor in the robot.
  const char *MOTOR_NAMES[] = {"motor1", "motor2", "motor3", "motor4"};
  
  // get motor device tags
  for (int i = 0; i < MOTOR_NUM; i++) {
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    v[i] = 0.0f;
  }
  
  FD_ZERO(&rfds);
  FD_SET(sfd, &rfds);

  // init linear_velocity untill we receive valid data from Supervisor.
  linear_velocity = &_linear_velocity[0] ;


  return true;
}
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{

  

  if (initialize( argc, argv))
  {
  
    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
    run();
  }

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

  return 0;
}
