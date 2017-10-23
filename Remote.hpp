#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
//#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>

#define MYPORT  80
#define QUEUE   20
#define BUFFER_SIZE 1024

#include <fstream>

#include <cstring>
#include <sstream>
#include <sys/stat.h> 

class Remote{
public:
    static int data[2];
    static void *Server(void *);
private:
    static std::string getHTML(const char[]);
    static std::string Response(const char[]);

    /* data */
};
int Remote::data[2];
std::string Remote::getHTML(const char filePath[]){
    std::ifstream fhtml(filePath);
    struct stat st;
    stat(filePath, &st);
    if(!fhtml){
        perror("打开文件出错");
        exit(1);
    }
    std::stringstream buffer;
    buffer<<"HTTP/1.1 200 OK\nContent-Type: text/html; charset=utf-8\nConnection: Keep-Alive\nContent-Length: ";
    buffer<<st.st_size<<"\n\n"<<fhtml.rdbuf();
    
    fhtml.close();
    return buffer.str();
}
std::string Remote::Response(const char str[]){
    std::string result;
    std::stringstream stream;
    stream<<str;
    std::string line[2];

    for(int i=0;i<2;i++){
        getline(stream,line[i],'|');
    }
    for(int i=0;i<2;i++){
        stream.clear();
        stream<<line[i];
        stream>>Remote::data[i];
    }

    result = "HTTP/1.1 200 OK\nContent-Type: application/json; charset=utf-8\nConnection: Keep-Alive\nContent-Length: 0\n\n";

    return result;
}
void *Remote::Server(void *){
    ///定义sockfd
    int server_sockfd = socket(AF_INET,SOCK_STREAM, 0);

    ///定义sockaddr_in
    struct sockaddr_in server_sockaddr;
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_port = htons(MYPORT);
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    int  reuse=1;
    if (setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
    {
                    perror("setsockopet error\n");
                    exit(1);
    }
    ///bind，成功返回0，出错返回-1
    if(bind(server_sockfd,(struct sockaddr *)&server_sockaddr,sizeof(server_sockaddr))==-1)
    {
        perror("bind");
        exit(1);
    }

    ///listen，成功返回0，出错返回-1
    if(listen(server_sockfd,QUEUE) == -1)
    {
        perror("listen");
        exit(1);
    }

    ///客户端套接字
    char recvBuffer[BUFFER_SIZE];
    std::string sendBuffer="";
    std::string s;
    struct sockaddr_in client_addr;
    socklen_t length = sizeof(client_addr);

    ///成功返回非负描述字，出错返回-1
    int conn = accept(server_sockfd, (struct sockaddr*)&client_addr, &length);
    if(conn<0){
        perror("connect");
        exit(1);
    }
    while(1){
        memset(recvBuffer,0,sizeof(recvBuffer));
        int len = recv(conn, recvBuffer, sizeof(recvBuffer),0);

        if(recvBuffer[0]!='G'||recvBuffer[1]!='E'||recvBuffer[2]!='T')continue;
        strtok(recvBuffer, "\r");//截取第一行
        recvBuffer[strlen(recvBuffer)-9]='\0';//去除 "HTTP/1.1"

//cout<<"--------------------------------------------"<<endl;
        //std::cout<<"recvBuffer:|"<<recvBuffer<<"|"<<std::endl;

        if(strlen(recvBuffer)==5)
        {
            sendBuffer=getHTML("Remote.html");
        }else{
            sendBuffer=Response(recvBuffer+6);
        }

//cout<<"sendBuffer:|"<<sendBuffer<<"|"<<endl;
        send(conn,sendBuffer.c_str(),sendBuffer.size(), 0);
        sendBuffer="";
    }
    close(conn);
    close(server_sockfd);

}
