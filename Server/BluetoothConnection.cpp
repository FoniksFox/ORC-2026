#include "BluetoothConnection.h"
#include <iostream>
#include <algorithm>

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#define INVALID_SOCKET -1
#elif _WIN32
#include <winsock2.h>
#include <ws2bth.h>
#pragma comment(lib, "ws2_32.lib")
#endif

BluetoothConnection::BluetoothConnection() : server_socket(INVALID_SOCKET), client_socket(INVALID_SOCKET) {
#ifdef _WIN32
  WSADATA wsaData;
  WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

BluetoothConnection::~BluetoothConnection() {
  disconnect();
#ifdef _WIN32
  WSACleanup();
#endif
}

bool BluetoothConnection::isConnected() const {
  return client_socket != INVALID_SOCKET;
}

void BluetoothConnection::setNonBlocking(uint64_t sock) {
#ifdef __linux__
  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#elif _WIN32
  u_long mode = 1;
  ioctlsocket(sock, FIONBIO, &mode);
#endif
}

// --- SERVER LOGIC ---
bool BluetoothConnection::startServer(uint8_t channel) {
#ifdef __linux__
  server_socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  struct sockaddr_rc loc_addr = { 0 };
  loc_addr.rc_family = AF_BLUETOOTH; 
  str2ba("00:00:00:00:00:00", &loc_addr.rc_bdaddr);
  loc_addr.rc_channel = channel; 

  if (bind(server_socket, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) < 0) return false;
  listen(server_socket, 1);
#elif _WIN32
  server_socket = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
  SOCKADDR_BTH loc_addr = { 0 };
  loc_addr.addressFamily = AF_BTH;
  loc_addr.btAddr = 0;
  loc_addr.port = channel;

  if (bind(server_socket, (struct sockaddr *)&loc_addr, sizeof(SOCKADDR_BTH)) == SOCKET_ERROR) return false;
  listen(server_socket, 1);
#endif
  return true;
}

bool BluetoothConnection::waitForClient() {
  if (server_socket == INVALID_SOCKET) return false;
  std::cout << "Waiting for incoming connection...\n";

#ifdef __linux__
  struct sockaddr_rc rem_addr = { 0 };
  socklen_t opt = sizeof(rem_addr);
  client_socket = accept(server_socket, (struct sockaddr *)&rem_addr, &opt);
#elif _WIN32
  SOCKADDR_BTH rem_addr = { 0 };
  int opt = sizeof(SOCKADDR_BTH);
  client_socket = accept(server_socket, (struct sockaddr *)&rem_addr, &opt);
#endif

  if (client_socket != INVALID_SOCKET) {
    setNonBlocking(client_socket); 
    return true;
  }
  return false;
}

// --- CLIENT LOGIC ---
bool BluetoothConnection::connect(const std::string& mac_address, uint8_t channel) {
#ifdef __linux__
  client_socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  struct sockaddr_rc addr = { 0 };
  addr.rc_family = AF_BLUETOOTH; 
  addr.rc_channel = channel; 
  str2ba(mac_address.c_str(), &addr.rc_bdaddr);

  if (::connect(client_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    client_socket = INVALID_SOCKET;
    return false;
  }
#elif _WIN32
  client_socket = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
  SOCKADDR_BTH addr = { 0 };
  addr.addressFamily = AF_BTH;
  addr.port = channel;

  // Convert MAC string "AA:BB:CC:DD:EE:FF" to Windows BTH_ADDR (uint64_t)
  int b[6];
  sscanf(mac_address.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]);
  addr.btAddr = ((BTH_ADDR)b[0] << 40) | ((BTH_ADDR)b[1] << 32) | ((BTH_ADDR)b[2] << 24) |
    ((BTH_ADDR)b[3] << 16) | ((BTH_ADDR)b[4] << 8)  | ((BTH_ADDR)b[5]);

  if (::connect(client_socket, (struct sockaddr *)&addr, sizeof(SOCKADDR_BTH)) == SOCKET_ERROR) {
    client_socket = INVALID_SOCKET;
    return false;
  }
#endif

  setNonBlocking(client_socket); 
  return true;
}

void BluetoothConnection::disconnect() {
#ifdef __linux__
  if (client_socket != INVALID_SOCKET) close(client_socket);
  if (server_socket != INVALID_SOCKET) close(server_socket);
#elif _WIN32
  if (client_socket != INVALID_SOCKET) closesocket(client_socket);
  if (server_socket != INVALID_SOCKET) closesocket(server_socket);
#endif
  client_socket = INVALID_SOCKET;
  server_socket = INVALID_SOCKET;
}

// --- DATA TRANSFER ---
bool BluetoothConnection::raw_send(const uint8_t* data, size_t length) {
#ifdef __linux__
  return write(client_socket, data, length) == length;
#elif _WIN32
  return ::send(client_socket, (const char*)data, length, 0) != SOCKET_ERROR;
#endif
}

uint8_t BluetoothConnection::calculateChecksum(const uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; i++) checksum ^= data[i];
  return checksum;
}

bool BluetoothConnection::processIncoming(uint8_t* out_packet, size_t struct_size, uint8_t expected_id) {
  if (!isConnected()) return false;

  uint8_t byte;
  int bytes_read = 0;

  // Drain the OS socket buffer completely
  while (true) {
#ifdef __linux__
    bytes_read = read(client_socket, &byte, 1);
#elif _WIN32
    bytes_read = recv(client_socket, (char*)&byte, 1, 0);
#endif
    if (bytes_read > 0) {
      rx_buffer.push_back(byte);
    } else {
      break; // No more bytes available right now
    }
  }

  // Sliding window parser
  if (rx_buffer.size() >= struct_size) {
    for (size_t i = 0; i <= rx_buffer.size() - struct_size; i++) {
      if (rx_buffer[i] == expected_id) {
        uint8_t expected_cs = calculateChecksum(&rx_buffer[i], struct_size - 1);
        if (expected_cs == rx_buffer[i + struct_size - 1]) {
          std::copy(rx_buffer.begin() + i, rx_buffer.begin() + i + struct_size, out_packet);
          rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + i + struct_size);
          return true;
        }
      }
    }
    if (rx_buffer.size() > 128) rx_buffer.clear(); // Flush if corrupted
  }
  return false;
}
