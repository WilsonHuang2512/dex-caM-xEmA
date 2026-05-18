#pragma once

int Init_TCP(int &server_sock);

int Interact_TCP(int server_sock);

int update_connect(int client_sock);

int update_disconnect(int client_sock);

int update_commands(int client_sock);

int unknown_command(int client_sock);

int kill_camera_server(int client_sock);

int get_camera_server_file(int client_sock);

int chmod_camera_server(int client_sock);

int reboot_device(int client_sock);

int setup_socket(int port);

int accept_new_connection(int server_sock);

int send_buffer(int sock, const char* buffer, int buffer_size);

int recv_buffer(int sock, char* buffer, int buffer_size);

int send_command(int sock, int command);

int recv_command(int sock, int* command);

int check_token(int client_sock);
