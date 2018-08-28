//
// Created by yoni on 27/08/18.
//

#ifndef PATHFINDER_SERVER_H
#define PATHFINDER_SERVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <string>
#include <stdlib.h>


using namespace std;
class Server{

public: explicit Server();
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);


    void waitForConnection();

    string readRequest();

    void send(string data);

};


#endif //PATHFINDER_SERVER_H
