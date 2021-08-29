#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>

#ifdef _WIN32
#include <winsock.h>
//#pragma comment(lib,"ws2_32.lib") //Winsock Library
#else
#include <arpa/inet.h> /* definition of inet_ntoa */
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif


int fd;
int sfd;

int create_socket_server(int port);
bool socket_cleanup();
int socket_accept(int server_fd);
bool socket_set_non_blocking(int fd);


