// Client side C/C++ program to demonstrate Socket programming
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 8080
typedef struct DetectedObject {
    int left;
    int right;
    int top;
    int bottom;
} DetectedObject;

typedef struct Packet {
    DetectedObject detectedObjects[50];
    int numOfObjects;
    unsigned long long timestamp;
} Packet;

  
int main(int argc, char const *argv[])
{
    Packet *packet;
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    int buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }
  
    memset(&serv_addr, '0', sizeof(serv_addr));
  
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
      
    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }
  
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }
    // send(sock , hello , strlen(hello) , 0 );
    // printf("Hello message sent\n");

    while (1) {
        valread = read(sock , buffer, 1024*4);
        if(buffer[0] != 0x12345678) continue;
        int numObjects = (int) buffer[3];
        printf("timestamp %u numObjects %d\n", (unsigned long long) buffer[1], (int) buffer[3]);
        int i=0;
        for (i; i < numObjects; i++) {
            printf("%d, %d, %d, %d\n", (int)buffer[4*i+4], (int)buffer[4*i+5],
            (int)buffer[4*i+6], (int) buffer[4*i+7]);
        }
        printf("\n");
    }
    return 0;
}