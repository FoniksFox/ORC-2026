#include "BluetoothConnection.h"
#include <iostream>
#include <string>
#include <sstream>
#include <conio.h> // Windows console I/O for _kbhit()
#include <thread>
#include <chrono>  

using namespace std;

// Windows helper function to check if keyboard has data without freezing
bool isKeyboardHit() {
  return _kbhit() != 0;
}

int main() {
  BluetoothConnection bt;
  string server_mac = "A0:B7:65:05:1A:2E"; 
  uint8_t channel = 1; 

  cout << "Connecting to ESP32 (" << server_mac << ") on channel " << (int)channel << "...\n";

  if (bt.connect(server_mac, channel)) {
    cout << "Connected!\n";
    cout << "Commands:\n";
    cout << "  update <rightV> <leftV>  (e.g., update 1.5 1.5)\n";
    cout << "  mode <0|1|2>             (e.g., mode 0)\n";

    while (true) {
      LogTelemetry data;

      // 1. Check for incoming telemetry
      if (bt.receive(data, LOG_TELEMETRY_ID)) {
        cout << "\n[TELEMETRY UPDATE]\n";
        cout << "Right Velocity: " << data.rightVelocity << '\n';
        cout << "Left Velocity : " << data.leftVelocity << '\n';
        cout << "Distance      : " << data.distance << '\n';
        cout << "------------------\n";
      }

      // 2. Check if the user pressed a key
      if (isKeyboardHit()) {
        string command;

        if (getline(cin, command)) {
          stringstream ss(command);
          string c;

          ss >> c;

          if (c == "update") {
            float rightV, leftV;
            if (ss >> rightV >> leftV) {
              UpdateVelocity cmd = {UPDATE_VELOCITY_ID, rightV, leftV, 0};
              if (bt.send(cmd)) {
                cout << "-> Sent Update: Right=" << cmd.rightVelocity << ", Left=" << cmd.leftVelocity << '\n';
              }
            } else {
              cout << "Invalid format. Use: update <rightV> <leftV>\n";
            }
          } 
          else if (c == "mode") {
            int m;
            if (ss >> m) {
              SetMode cmd = {SET_MODE_ID, m, 0};
              if (bt.send(cmd)) {
                cout << "-> Sent Mode Change: " << cmd.mode << '\n';
              }
            } else {
              cout << "Invalid format. Use: mode <0|1|2>\n";
            }
          } 
          else {
            cout << "Unknown command.\n";
          }
        }
      }

      // Sleep 1ms to prevent maxing out the CPU
      this_thread::sleep_for(chrono::milliseconds(1));
    }
  } else {
    cerr << "Connection failed. Is the ESP32 turned on and paired to this PC?\n";
  }

  return 0;
}
