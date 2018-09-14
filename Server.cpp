//
// Created by yoni on 27/08/18.
//

#include "Server.h"
#include <vector>
#include <list>
#include <map>
#include <set>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 20000
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"
#include <unistd.h>
#include <stdio.h>

using namespace std;

Server::Server()
{
    addrlen = sizeof(address);
    cout << "socket is ready for action!" << endl;
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *) &address,
             sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
}


void Server::waitForConnection()
{
    if ((new_socket = accept(server_fd, (struct sockaddr *) &address, (socklen_t *) &addrlen)) < 0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
        // return false;
    }

    cout << "waiting..." << endl; //todo

}

string Server::readRequest()
{
    char buffer[1024] = {0};
    int param_read = read(new_socket, buffer, 1024);
    //cout << param_read << endl;
    cout << buffer << endl;
    return buffer;
}

void Server::send(string data)
{
    ::send(new_socket, data.c_str(), data.length(), 0); // todo check that works
}
