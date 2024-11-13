// g++ gnss_reader.cpp -o gnss_reader
//

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <vector>
#include <ostream>
#include <sstream>
#include <iostream>

// GNRMC NMEA has its own version of essential gps pvt (position,
// velocity, time) data. It is called RMC, The Recommended Minimum,
enum GNRMC_Fields 
{
   $GNRMC               = 0,
   UTC_TIME             = 1,  // Format is hhmmss.sss
   POSITIONING_STATUS   = 2,  // A=effective positioning, V=invalid positioning
   LATITUDE             = 3,  // The format is ddmm.mmmmmmm
   LATITUDE_HEMISPHERE  = 4,  // N or S (north latitude or south latitude)
   LONGITUDE            = 5,  // the format is dddmm.mmmmmmm
   LONGITUDE_HEMISPHERE = 6,  // E or W (east longitude or west longitude)
   GROUND_SPEED         = 7,
   GROUND_HEADING       = 8,  // (take true north as the reference datum)
   UTC_DATE             = 9,  // Format is ddmmyy (day, month, year)
   MAGNETIC_DECLINATION = 10, //(000.0~180.0 degrees)
   MAGNETIC_DECLINATION_DIRECTION = 11, // E (east) or W (west)
   MODE_INDICATION                = 12, // A=autonomous positioning, 
                                        // D=differential, E=estimation, 
                                        // N=invalid data
};

const char* stringOf(const GNRMC_Fields field)
{
   switch(field)
   {
      case $GNRMC: return "$GNRMC";
      case UTC_TIME: return "UTC_TIME";
      case POSITIONING_STATUS: return "POSITIONING_STATUS";
      case LATITUDE: return "LATITUDE";
      case LATITUDE_HEMISPHERE: return "LATITUDE_HEMISPHERE";
      case LONGITUDE: return "LONGITUDE";
      case LONGITUDE_HEMISPHERE: return "LONGITUDE_HEMISPHERE";
      case GROUND_SPEED: return "GROUND_SPEED";
      case GROUND_HEADING: return "GROUND_HEADING";
      case UTC_DATE: return "UTC_DATE";
      case MAGNETIC_DECLINATION: return "MAGNETIC_DECLINATION";
      case MAGNETIC_DECLINATION_DIRECTION: return "MAGNETIC_DECLINATION_DIRECTION";
      case MODE_INDICATION: return "MODE_INDICATION";
      default: return "UNK";
   }
}

void displayGNRMC(std::vector<std::string> tokens)
{
   int index(0);
   for (const auto& elem : tokens)
   {
      std::cout << stringOf(GNRMC_Fields(index++)) << " => " << elem << std::endl;
   }
   
}

std::vector<std::string> tokensOf(const std::string& str)
{
   std::cout << "NMEA Data: " << str;

   std::vector<std::string> tokens;
   std::stringstream ss(str);
   std::string token;
   
   while(std::getline(ss, token, ','))
   {
      tokens.push_back(token);
   }
   
   if (tokens[0] == "$GNRMC") displayGNRMC(tokens);

   return tokens;
}

int main() {
    // Open the serial port
    int serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port == -1) {
        perror("Unable to open serial port");
        return 1;
    }

    // Configure the serial port
    struct termios options;
    tcgetattr(serial_port, &options);
    cfsetispeed(&options, B57600);  // Set baud rate to 9600
    cfsetospeed(&options, B57600);
    options.c_cflag |= (CLOCAL | CREAD);  // Enable the receiver and set local mode
    options.c_cflag &= ~CSIZE;            // Mask character size bits
    options.c_cflag |= CS8;               // Select 8 data bits
    options.c_cflag &= ~PARENB;           // No parity
    options.c_cflag &= ~CSTOPB;           // 1 stop bit
    tcsetattr(serial_port, TCSANOW, &options);

    // Buffer to store received data
    char read_buf[256];
    memset(read_buf, 0, sizeof(read_buf));

    while (1) {
        // Read up to 255 characters from the serial port
        int num_bytes = read(serial_port, read_buf, sizeof(read_buf) - 1);

        if (num_bytes < 0) 
        {
            perror("Error reading from serial port");
            // break;
        } else if (num_bytes > 0) 
        {
            read_buf[num_bytes] = '\0';  // Null-terminate the string
            
            //printf("Received: %s", read_buf);

            // Look for GNRMC or GNGGA sentences in the buffer
            if (strstr(read_buf, "$GNRMC") || 
                strstr(read_buf, "$GNGGA") ||
                strstr(read_buf, "$GNGSA")) 
            {
                //printf("NMEA Data: %s", read_buf);
                
                // Here, you could parse the NMEA sentence further if needed.
                
                std::string buffer(read_buf, num_bytes);
                tokensOf(buffer);
            }
        }
        usleep(100000);  // Small delay to avoid overloading CPU
    }

    // Close the serial port
    close(serial_port);
    return 0;
}
