#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <iostream>
#include <errno.h> 
#include <cassert>
#include <random>
#include "../firmware/protocol.h"
#include "easylogging++.h"
#include "update_opencam3d.h"

long long current_token = 0;
bool connected = false;

std::random_device rd;
std::mt19937 rand_num(rd());

int Init_TCP(int &server_sock)
{
    do
    {
        server_sock = setup_socket(UPDATE_PORT);
        sleep(1);
    }
    while(server_sock == DF_FAILED);
    std::cout << "update server listening" << std::endl;

    return 0;
}

int Interact_TCP(int server_sock)
 {
    int ret = DF_FAILED;
    int client_sock = accept_new_connection(server_sock);

    if(client_sock != -1)
    {
        ret = update_commands(client_sock);
    }

    return ret;
}

int update_commands(int client_sock)
{
    int command;
    int ret = recv_command(client_sock, &command); 
    LOG(INFO)<<"update firmware command:"<<command;
    
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"update firmware connection command not received";
	    close(client_sock);
        return ret;
    }

    switch(command)
    {
        case DF_CMD_CONNECT:
            LOG(INFO)<<"DF_CMD_CONNECT";
            update_connect(client_sock);
            break;
        case DF_CMD_DISCONNECT:
            LOG(INFO)<<"DF_CMD_DISCONNECT";
            update_disconnect(client_sock);
            break;
        case DF_CMD_KILL_CAMERA_SERVER:;
        	LOG(INFO)<<"DF_CMD_KILL_CAMERA_SERVER";
            ret = kill_camera_server(client_sock);
            break;

        case DF_CMD_GET_CAMERA_SERVER:
        	LOG(INFO)<<"DF_CMD_GET_CAMERA_SERVER";
            ret = get_camera_server_file(client_sock);
            break;

        case DF_CMD_CHMOD_CAMERA_SERVER:
        	LOG(INFO)<<"DF_CMD_CHMOD_CAMERA_SERVER";
            ret = chmod_camera_server(client_sock);
            break;

        case DF_CMD_REBOOT_DEVICE:
        	LOG(INFO)<<"DF_CMD_REBOOT_DEVICE";
            ret = reboot_device(client_sock);
            break;
        default:
            LOG(INFO)<<"DF_CMD_UNKNOWN";
            unknown_command(client_sock);
            break;
    }

    close(client_sock);
    return ret;
}

long long generate_token()
{
    long long token = rand_num();
    return token;
}

int update_connect(int client_sock)
{
    int ret;
    if (connected)
    {
        std::cout<<"update new connection rejected"<<std::endl;
        return send_command(client_sock, DF_CMD_REJECT);
    }
    else
    {
        ret = send_command(client_sock, DF_CMD_OK);
        if(ret == DF_FAILED)
        {
            return DF_FAILED;
        }

        long long token = generate_token();
        ret = send_buffer(client_sock, (char*)&token, sizeof(token));
        if(ret == DF_FAILED)
        {
            return DF_FAILED;
        }
        connected = true;
        current_token = token;
        
        return DF_SUCCESS;
    }
}

int update_disconnect(int client_sock)
{
    std::cout<<"update disconnect"<<std::endl;
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));
    std::cout<<"token "<<token<<" trying to disconnect"<<std::endl;
    if(ret == DF_FAILED)
    {
	    return DF_FAILED;
    }

    if(token == current_token)
    {
        connected = false;
        current_token = 0;
        std::cout<<"client token="<<token<<" disconnected"<<std::endl;
        ret = send_command(client_sock, DF_CMD_OK);
        if(ret == DF_FAILED)
        {
            return DF_FAILED;
        }
    }
    else
    {
        std::cout<<"disconnect rejected"<<std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        if(ret == DF_FAILED)
        {
            return DF_FAILED;
        }
    }

    return DF_SUCCESS;
}

int unknown_command(int client_sock)
{
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));
    if(ret == DF_FAILED) 
    {
    	return DF_FAILED;
    }

    if(token == current_token)
    {
        ret = send_command(client_sock, DF_CMD_UNKNOWN);
        return DF_SUCCESS;
    }
    else
    {
        std::cout<<"update firmware reject"<<std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        return DF_FAILED;
    }
}

int kill_camera_server(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        LOG(INFO)<<"check_token error!";
	    return DF_FAILED;
    }

    system("sudo kill -9 $(pidof camera_server)");

    int feedback = 1010;
    int ret = send_buffer(client_sock, (char*)(&feedback), sizeof(feedback));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int get_camera_server_file(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED) {
	    return DF_FAILED;
    }

    unsigned int file_size;
    int ret = recv_buffer(client_sock, (char*)(&file_size), sizeof(file_size));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"recv error, close this connection!\n";
	    return DF_FAILED;
    }

    char string[100] = {'\0'};
    sprintf(string, "update camera_file size: 0x%X", file_size);

    char *org_buffer = new char[file_size];
    ret = recv_buffer(client_sock, org_buffer, file_size);
    if (ret == DF_FAILED)
    {
        delete [] org_buffer;
        LOG(INFO)<<"recv error, close this connection!\n";
	    return DF_FAILED;
    }

    system("rm -f camera_server");

	FILE* fw;
	fw = fopen("camera_server", "wb");
    if (fw != NULL) {
		fwrite(org_buffer, 1, file_size, fw);
		fclose(fw);
	}
	else {
        LOG(INFO)<< "save camera_server file fail";
	}

    delete [] org_buffer;

    return DF_SUCCESS;
}

int chmod_camera_server(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    system("sudo chown dexforce:dexforce camera_server");
    system("chmod +x camera_server");

    int feedback = 2020;
    int ret = send_buffer(client_sock, (char*)(&feedback), sizeof(feedback));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    return DF_SUCCESS;
}

int reboot_device(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    system("sudo reboot");

    int feedback = 3030;
    int ret = send_buffer(client_sock, (char*)(&feedback), sizeof(feedback));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int setup_socket(int port)
{
    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(server_sock<0)
    {
        perror("ERROR: socket()");
        exit(0);
    }

    int flags = 3;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPIDLE, (void*)&flags, sizeof(flags));
    flags = 3;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPCNT, (void*)&flags, sizeof(flags));
    flags = 1;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPINTVL, (void*)&flags, sizeof(flags));


    //将套接字和IP、端口绑定
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充
    serv_addr.sin_family = AF_INET;  //使用IPv4地址
    serv_addr.sin_addr.s_addr = INADDR_ANY;  //具体的IP地址
    serv_addr.sin_port = htons(port);  //端口
    int ret = bind(server_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    if(ret==-1)
    {
        printf("bind ret=%d, %s\n", ret, strerror(errno));
        close(server_sock);
        return DF_FAILED;
    }

    //进入监听状态，等待用户发起请求
    ret = listen(server_sock, 1);
    if(ret == -1)
    {
        printf("listen ret=%d, %s\n", ret, strerror(errno));
        close(server_sock);
        return DF_FAILED;
    }
    return server_sock;
}

int accept_new_connection(int server_sock)
{
    //std::cout<<"listening"<<std::endl;
    //接收客户端请求
    struct sockaddr_in clnt_addr;
    socklen_t clnt_addr_size = sizeof(clnt_addr);
    int client_sock = accept(server_sock, (struct sockaddr*)&clnt_addr, &clnt_addr_size);

    //print address
    char buffer[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &clnt_addr.sin_addr, buffer, sizeof(buffer));

    struct timeval timeout = {1,0};
    int ret = setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    ret = setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
    //int flags = 1;
    //setsockopt(client_sock, SOL_SOCKET, SO_KEEPALIVE, (void*)&flags, sizeof(flags));

    //std::cout<<"accepted connection from "<<buffer<<std::endl;
    
    return client_sock;
}

int send_buffer(int sock, const char* buffer, int buffer_size)
{
    int size = 0;
    int ret = send(sock, (char*)&buffer_size, sizeof(buffer_size), MSG_NOSIGNAL);
    LOG(INFO)<<"send buffer_size ret="<<ret;
    if (ret == -1)
    {
        return DF_FAILED;
    }

    int sent_size = 0;
    ret = send(sock, buffer, buffer_size, MSG_NOSIGNAL);
    LOG(INFO)<<"send buffer ret="<<ret;
    if (ret == -1)
    {
        return DF_FAILED;
    }
    sent_size += ret;

    while(sent_size != buffer_size)
    {
	    buffer += ret;
	    LOG(INFO)<<"sent_size="<<sent_size;
	    ret = send(sock, buffer, buffer_size-sent_size, MSG_NOSIGNAL);
        LOG(INFO)<<"ret="<<ret;
        if (ret == -1)
        {
            return DF_FAILED;
        }
	    sent_size += ret;
    }

    return DF_SUCCESS;
}

int recv_buffer(int sock, char* buffer, int buffer_size)
{
    int size = 0;
    int ret = recv(sock, (char*)&size, sizeof(size), 0);
    assert(buffer_size >= size);
    int n_recv = 0;
    ret = DF_SUCCESS;

    while (ret != -1)
    {
        ret = recv(sock, buffer, buffer_size, 0);
        //std::cout << "ret="<<ret << std::endl;
        if (ret > 0)
        {
            buffer_size -= ret;
            n_recv += ret;
            buffer += ret;
        }

        if (buffer_size == 0)
        {
            assert(n_recv == size);
            return DF_SUCCESS;
        }
    }
    return DF_FAILED;
}

int send_command(int sock, int command)
{
    return send_buffer(sock, (const char*)&command, sizeof(int));
}

int recv_command(int sock, int* command)
{
    return recv_buffer(sock, (char*)command, sizeof(int));
}

int check_token(int client_sock)
{
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));

    if(ret == DF_FAILED)
    {
	    return DF_FAILED;
    }

    if(token == current_token)
    {
        ret = send_command(client_sock, DF_CMD_OK);
        return DF_SUCCESS;
    }
    else
    {
        std::cout<<"reject"<<std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        return DF_FAILED;
    }
}