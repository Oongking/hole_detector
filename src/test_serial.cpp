 // Include the ROS library
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <sstream>
// #include <iomanip>
uint8_t pro_address[4];
// Function to convert a hexadecimal string to a byte array
std::vector<uint8_t> hexStringToByteArray(const std::string& hexString) 
{ 
    std::vector<uint8_t> byteArray; 
  
    // Loop through the hex string, two characters at a time 
    for (size_t i = 0; i < hexString.length(); i += 2) { 
        // Extract two characters representing a byte 
        std::string byteString = hexString.substr(i, 2); 
  
        // Convert the byte string to a uint8_t value 
        uint8_t byteValue = static_cast<uint8_t>( 
            stoi(byteString, nullptr, 16)); 
  
        // Add the byte to the byte array 
        byteArray.push_back(byteValue); 
    } 
  
    return byteArray; 
} 

uint8_t calculateXorChecksum(std::stringstream& ss) {
    uint8_t checksum = 0;
    char byte;

    // XOR Checksum
    while (ss.get(byte)) {
        checksum ^= byte;
    }

    return checksum;
}

void serial_format(uint8_t* length,uint8_t com[], uint8_t data[]){
    std::stringstream ss;
    std::stringstream sent;
    char byte;


    ss  << static_cast<char>(*length)
        << static_cast<char>(com[0])
        << static_cast<char>(com[1]);
    
    if (*length != 0){
        for (int i = 0; i< (*length*2);i++){
            ss << static_cast<char>(data[i]);
        }
    } 
    
    // Calculate checksum
    uint8_t checksum = calculateXorChecksum(ss);

    sent    << static_cast<char>(0x02) 
            << ss.str() 
            << static_cast<char>(0x03) 
            << static_cast<char>(checksum);



    // Print content as hex
    std::cout << std::hex << std::setfill('0');    
    while (sent.get(byte)) {
        std::cout << std::setw(2) << static_cast<int>(static_cast<unsigned char>(byte)) << " ";
    }
    // Clear the end-of-file state
    sent.clear(); 
    sent.seekg(0);
    std::cout << std::endl;

}


void innerTrig(uint8_t onoff)
    {

        uint8_t length = 0x01;
        uint8_t com[2] = {0xc0,0x05};
        uint8_t data[2] = {0x00,onoff};
        serial_format(&length,com,data);


        // std::cout << ss.str() << std::endl;
    }

void innerHole(uint8_t onoff)
    {

        uint8_t length = 0x01;
        uint8_t com[2] = {0xa0,0x15};
        uint8_t data[2] = {0x00,onoff};
        serial_format(&length,com,data);


        // std::cout << ss.str() << std::endl;
    }

void getProfileAddress()
    {

        uint8_t length = 0x00;
        uint8_t com[2] = {0x40,0x0B};
        uint8_t data[1] = {0x00};
        serial_format(&length,com,data);


        // std::cout << ss.str() << std::endl;
    }

void getProfileSize()
    {
        uint8_t length = 0x03;
        uint8_t com[2] = {0x00,0x02};
        uint8_t data[6] = {pro_address[0],
                            pro_address[1],
                            pro_address[2],
                            pro_address[3],
                            0x01,
                            0x11,};
        serial_format(&length,com,data);
    }

void getProfile()
    {
        uint8_t length = 0x03;
        uint8_t com[2] = {0x00,0x02};
        uint8_t data[6] = {pro_address[0],
                            pro_address[1],
                            pro_address[2],
                            pro_address[3],
                            0x7E,
                            0x21,};
        serial_format(&length,com,data);
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_serial");
    ros::NodeHandle nh;

// ################ Example hexadecimal string ###############
    // std::string hexString = "48656C6C6F20576F726C64"; // "Hello World" in hexadecimal

    // std::vector<uint8_t> byteArray1 
    //     = hexStringToByteArray(hexString); 
  
    // // Print the input and output 
    // std::cout << "Input Hex String: " << hexString << std::endl; 
    // std::cout << "Output Byte Array: "; 
    // for (uint8_t byte : byteArray1) { 
    //     std::cout << static_cast<int>(byte) << " "; 
    // } 
// ################ Example hexadecimal string ###############


    innerTrig(true);
    innerHole(true);
    getProfileAddress();
    getProfileSize();
    getProfile();

    return 0;
}