//
// Created by yoni on 27/08/18.
//

#ifndef PATHFINDER_SERVER_H
#define PATHFINDER_SERVER_H

#include <sys/socket.h>
#include <sys/socket.h>
#include <string>

using namespace std;
class Server{

public: explicit Server();
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);


    void waitForConnection();

    string readRequest();

};


#endif //PATHFINDER_SERVER_H
