#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include "geometry_msgs/Point32.h"

#include "AsyncSerial.h"



class FastusProxy
{
public:
    FastusProxy(std::string port_name, uint32_t baudrate, ros::NodeHandle *_nh) : rxBuff("")
    {
       

        serial = boost::make_shared<CallbackAsyncSerial>(port_name, baudrate);
        serial->setCallback(boost::bind(&FastusProxy::received, this, _1, _2));
        // _cam_state_timer = _nh->createTimer(ros::Duration(0.02), &FastusProxy::pubSerial, this);
        point_pub = _nh->advertise<geometry_msgs::Point32>("point_array", 10);
    }
    
    void run(){
        

        res = innerTrig(true);
        std::cerr << "innerTrig : " << res << std::endl;

        res = getProfileAddress();
        std::cerr << "getProfileAddress : " << res << std::endl;
        std::cout << "ProfileAddress  : ";
        printUInt32AsBytes(pro_address.asUInt);
        std::cout << std::endl;

        res = getProfileSize();
        std::cerr << "getProfileSize : " << res << std::endl;
        

        std::cout << std::endl;

        res = getProfile();

        std::cout << "1getProfileSize  : "<< pro_size.asUInt/32<< " point" << std::endl;
        // printUInt16AsBytes(pro_size.asUInt);
        int point_size = sizeof(profile_data);

        if (res){

            for (int i = 0; i < 10;i++ ){
                std::cout << std::dec<< "point "<< i;
                std::cout << "\t x:"<< profile_data[i].x.asInt;
                std::cout << "\t y:"<< profile_data[i].y.asInt;
                std::cout << "\t point : "<< i <<"/"<<point_size;
                std::cout << std::endl;
            }
        }

        std::vector<geometry_msgs::Point32> points;
        geometry_msgs::Point32 point1;
        for (int i = 0; i < point_size;i++ ){
            point1.x = profile_data[i].x.asInt;
            point1.y = profile_data[i].y.asInt;
            points.push_back(point1);
        }



        // ros::Duration(5).sleep();
        // res = getProfileAddress();
        // std::cerr << "getProfileAddress : " << res << std::endl;
        // std::cout << "ProfileAddress  : ";
        // printUInt32AsBytes(pro_address.asUInt);
        // std::cout << std::endl;

        // res = getProfile();
        // std::cout << "1getProfileSize  : "<< pro_size.asUInt/32<< " point" << std::endl;
        // // printUInt16AsBytes(pro_size.asUInt);

        // if (res){

        //     for (int i = 0; i < 10;i++ ){
        //         std::cout << std::dec<< "point "<< i;
        //         std::cout << "\t x:"<< profile_data[i].x.asInt;
        //         std::cout << "\t y:"<< profile_data[i].y.asInt;
        //         std::cout << "\t point : "<< i <<"/"<<point_size;
        //         std::cout << std::endl;
        //     }
        // }

        // for (int i = 0; i < point_size;i++ ){
        //     point1.x = profile_data[i].x.asInt;
        //     point1.y = profile_data[i].y.asInt;
        //     points.push_back(point1);
        // }

        // ros::Duration(5).sleep();
        // res = getProfileAddress();
        // std::cerr << "getProfileAddress : " << res << std::endl;
        // std::cout << "ProfileAddress  : ";
        // printUInt32AsBytes(pro_address.asUInt);
        // std::cout << std::endl;

        // res = getProfile();
        // std::cout << "1getProfileSize  : "<< pro_size.asUInt/32<< " point" << std::endl;
        // // printUInt16AsBytes(pro_size.asUInt);

        // if (res){

        //     for (int i = 0; i < 10;i++ ){
        //         std::cout << std::dec<< "point "<< i;
        //         std::cout << "\t x:"<< profile_data[i].x.asInt;
        //         std::cout << "\t y:"<< profile_data[i].y.asInt;
        //         std::cout << "\t point : "<< i <<"/"<<point_size;
        //         std::cout << std::endl;
        //     }
        // }

        for (int i = 0; i < point_size;i++ ){
            point1.x = profile_data[i].x.asInt;
            point1.y = profile_data[i].y.asInt;
            points.push_back(point1);
        }


        std::cout << "start pub : " << sizeof(points) << std::endl;

        ros::Rate loop_rate(1); // Adjust the publishing frequency as needed
        while (ros::ok()) {
            // Publish each point
            for (const auto &point : points) {
                point_pub.publish(point);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        



        // res = getProfile();

        // std::cout << "2getProfileSize  : "<< pro_size.asUInt/32<< " point" << std::endl;

        // if (res){
        //     int point_size = sizeof(profile_data);

        //     for (int i = 0; i < 10;i++ ){
        //         std::cout << std::dec<< "point "<< i;
        //         std::cout << "\t x:"<< profile_data[i].x.asInt;
        //         std::cout << "\t y:"<< profile_data[i].y.asInt;
        //         std::cout << "\t point : "<< i <<"/"<<point_size;
        //         std::cout << std::endl;
        //     }
        // }




        // res = getProfile();

        // std::cout << "3getProfileSize  : "<< pro_size.asUInt/32<< " point" << std::endl;

        // if (res){
        //     int point_size = sizeof(profile_data);

        //     for (int i = 0; i < 10;i++ ){
        //         std::cout << std::dec<< "point "<< i;
        //         std::cout << "\t x:"<< profile_data[i].x.asInt;
        //         std::cout << "\t y:"<< profile_data[i].y.asInt;
        //         std::cout << "\t point : "<< i <<"/"<<point_size;
        //         std::cout << std::endl;
        //     }
        // }





        // pro_address1 = pro_address;

        // res = getProfileAddress();
        // std::cerr << "getProfileAddress : " << res << std::endl;
        // std::cout << "ProfileAddress  : ";
        // printUInt32AsBytes(pro_address.asUInt);
        // std::cout << std::endl;
        // pro_address2 = pro_address;

        // res = getProfileAddress();
        // std::cerr << "getProfileAddress : " << res << std::endl;
        // std::cout << "ProfileAddress  : ";
        // printUInt32AsBytes(pro_address.asUInt);
        // std::cout << std::endl;
        // std::cout << "ProfileAddress1  : ";
        // printUInt32AsBytes(pro_address1.asUInt);
        // std::cout << std::endl;
        // std::cout << "ProfileAddress2  : ";
        // printUInt32AsBytes(pro_address2.asUInt);
        // std::cout << std::endl;


        // res = innerHole(true);
        // std::cout << "innerHole : " << res << std::endl;

    }

private:

    ros::Timer _cam_state_timer;
    boost::shared_ptr<CallbackAsyncSerial> serial;
    ros::Publisher point_pub;

    enum mode{
        idle = 0,
        sent_Trig = 1,
        sent_Hole = 2,
        sent_getProAddress = 3,
        sent_getProSize = 4,

        get_Trig = 5,
        get_Hole = 6,

        sent_getPro = 10,

    } type_tag;

    union Int16ToByte
    {
        int16_t asInt;
        uint8_t asByte[2];
    };

    union UInt16ToByte
    {
        uint16_t asUInt;
        uint8_t asByte[2];
    };

    union UInt32ToByte
    {
        uint32_t asUInt;
        uint8_t asByte[4];
    };

    union FloatToByte
    {
        float asFloat;
        uint8_t asByte[4];
    };
    struct point
    {
        Int16ToByte x;
        Int16ToByte y;
    };

    point profile_data[126];
    std::string rxBuff;
    UInt32ToByte pro_address;
    UInt16ToByte pro_size;

    UInt16ToByte tag_reply;
    double_t sleep_time = 2.0;

    
    
    bool res = false;
    

    
    uint8_t calculateXorChecksum(std::stringstream& ss) {

        // char p;
        // // ####### Print content as hex #######
        // std::cout << std::hex << std::setfill('0');    
        // while (ss.get(p)) {
        //     std::cout << std::setw(2) << static_cast<int>(static_cast<unsigned char>(p)) << " ";
        // }
        // // Clear the end-of-file state
        // ss.clear(); 
        // ss.seekg(0);
        // std::cout << std::endl;

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
        
        // ####### Calculate checksum #######
        uint8_t checksum = calculateXorChecksum(ss);

        sent    << static_cast<char>(0x02) 
                << ss.str() 
                << static_cast<char>(0x03) 
                << static_cast<char>(checksum);

        serial->writeString(sent.str());

        double start_secs =ros::Time::now().toSec();

        while((ros::Time::now().toSec()-start_secs) < sleep_time && type_tag != idle){

        }

        // ros::Duration(sleep_time).sleep();

        // ####### Print content as hex #######
        // std::cout << std::hex << std::setfill('0');    
        // while (sent.get(byte)) {
        //     std::cout << std::setw(2) << static_cast<int>(static_cast<unsigned char>(byte)) << " ";
        // }
        // // Clear the end-of-file state
        // sent.clear(); 
        // sent.seekg(0);
        // std::cout << std::endl;



    }


    bool innerTrig(uint8_t onoff)
        {   
            bool camstate = getTrig();

            if (camstate != onoff){
                type_tag = sent_Trig;
                std::cout << "trigger" << std::endl;
                
                while(ros::ok() && type_tag !=0){
                    uint8_t length = 0x01;
                    uint8_t com[2] = {0xc0,0x05};
                    uint8_t data[2] = {0x00,onoff};
                    serial_format(&length,com,data);

                }
            }else{
                std::cout << "trigger already : " << bool(onoff) << std::endl;
                
            }

            return onoff;
            // std::cout << ss.str() << std::endl;
        }
    
    bool getTrig()
        {
            type_tag = get_Trig;
            std::cout << "get trigger" << std::endl;
            
            while(ros::ok() && type_tag !=0){
                uint8_t length = 0x00;
                uint8_t com[2] = {0xc0,0x06};
                uint8_t data[2] = {0x00,0x00};
                serial_format(&length,com,data);

            }
            if (tag_reply.asByte[0]==0x00){
                if (tag_reply.asByte[1]==0x01){
                    return true;
                }else if(tag_reply.asByte[1]==0x00){
                    return false;
                }
            }
      
            // std::cout << ss.str() << std::endl;
        }

    bool innerHole(uint8_t onoff)
        {
            type_tag = sent_Hole;
            while(ros::ok() && type_tag !=0){
                uint8_t length = 0x01;
                uint8_t com[2] = {0xa0,0x15};
                uint8_t data[2] = {0x00,onoff};
                serial_format(&length,com,data);
            }
            if (type_tag ==0){
                return true;
            }else{
                return false;
            }

            // std::cout << ss.str() << std::endl;
        }

    bool getProfileAddress()
        {

            type_tag = sent_getProAddress;
            std::cout << "get ProfileAddress" << std::endl;

            while(ros::ok() && type_tag !=0){

                uint8_t length = 0x00;
                uint8_t com[2] = {0x40,0x0B};
                uint8_t data[1] = {0x00};
                serial_format(&length,com,data);

            
            }

            if (type_tag ==0){
                return true;
            }else{
                return false;
            }

            // std::cout << ss.str() << std::endl;
        }

    bool getProfileSize()
        {

            type_tag = sent_getProSize;
            while(ros::ok() && type_tag !=0){
                uint8_t length = 0x03;
                uint8_t com[2] = {0x00,0x02};
                uint8_t data[6] = {pro_address.asByte[3],
                                    pro_address.asByte[2],
                                    pro_address.asByte[1],
                                    pro_address.asByte[0],
                                    0x01,
                                    0x11,};
                serial_format(&length,com,data);
            
            }
            if (type_tag ==0){
                return true;
            }else{
                return false;
            }

        }

    bool getProfile()
        {
            type_tag = sent_getPro;
            pro_address.asUInt += 4; // skip header

            while(ros::ok() && type_tag !=0){
                uint8_t length = 0x03;
                uint8_t com[2] = {0x00,0x02};
                uint8_t data[6] = {pro_address.asByte[3],
                                    pro_address.asByte[2],
                                    pro_address.asByte[1],
                                    pro_address.asByte[0],
                                    0x7E,
                                    0x21,};
                serial_format(&length,com,data);
            }
            if (type_tag ==0){
                return true;
            }else{
                return false;
            }

        }

    void pubSerial(const ros::TimerEvent &event)
    {
        std::stringstream ss;

        ss << static_cast<char>(0x02)
            << static_cast<char>(0x00)
            << static_cast<char>(0xc0)
            // << static_cast<char>(0x10)
            // << static_cast<char>(0x0)
            // << static_cast<char>(0x02)
            // << static_cast<char>(0x00)
            << static_cast<char>(0x10)
            << static_cast<char>(0x03)
            << static_cast<char>(0xD0);

        serial->writeString(ss.str());


        // std::cout << ss.str() << std::endl;
    }
    
    
    std::string byteToHex(unsigned char byte) {
        std::stringstream ss;
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
        return ss.str();
    }

    void printUInt32AsBytes(uint32_t value) {

        std::cout << "Byte 32int: ";
        for (int i = 0; i < 4; ++i) { // 4 bytes in a uint32_t
            // Extract the i-th byte using bitwise AND with a mask
            uint8_t byte = (value >> (i * 8)) & 0xFF;
            std::cout << static_cast<int>(byte);
        }
        std::cout << std::endl;
    }

    void printUInt16AsBytes(uint16_t value) {
        std::cout << "Byte 16int: ";
        for (int i = 0; i < 2; ++i) { // 4 bytes in a uint32_t
            // Extract the i-th byte using bitwise AND with a mask
            uint8_t byte = (value >> (i * 8)) & 0xFF;
            std::cout << static_cast<int>(byte);
        }
        std::cout << std::endl;
    }

    void print_rxBuff(std::string rxBuf,std::string title){
        int length = rxBuff[1]*2; 
        std::cout << "checksum require : "<< 6+length << std::endl;
        std::cout << "checksum print : "<< rxBuff.size() << std::endl;
        std::cout << title;
        for (size_t j = 0; j < 6+length; j++){
            std::cout << byteToHex(rxBuff[j]) << " ";
        }
        std::cout << " : end" << std::endl;
    }

    void received(const char *data, size_t len)
    {

        std::string s(data, data + len);
        rxBuff += s;
        std::cout << "rxBuff size: "<< std::dec << rxBuff.size() << std::endl;
        

        if (type_tag !=0){
        
            if (rxBuff.size() > 600)
            {
                rxBuff.erase(0, (rxBuff.size()-300));
                // ROS_WARN("You fuck up");

                std::cerr<< "size : "<< rxBuff.size() << "low-level feedback is overflow" << " mode : "<< type_tag << std::endl;
            }else if (rxBuff.size() > 6)
            {
                for (size_t i = 0; i < rxBuff.size(); i++){

                    if (rxBuff[0] != 0x02 && rxBuff.size() > 1 && type_tag !=10) { // Check for STX
                        rxBuff.erase(0, 1);
                    }else if(type_tag == 10 &&
                            (rxBuff[0] != static_cast<char>(0x02) || 
                            rxBuff[2] != static_cast<char>(0x00) || 
                            rxBuff[3] != static_cast<char>(0x02) ||
                            rxBuff[4] != static_cast<char>(pro_address.asByte[3]) ||
                            rxBuff[5] != static_cast<char>(pro_address.asByte[2]) ||
                            rxBuff[6] != static_cast<char>(pro_address.asByte[1]) ||
                            rxBuff[7] != static_cast<char>(pro_address.asByte[0]) )){
                                
                                std::cout << "condition : "
                                << byteToHex(0x02) << (rxBuff[0] != static_cast<char>(0x02)) << " "
                                << "Any" << " "
                                << byteToHex(0x00) << (rxBuff[2] != static_cast<char>(0x00)) << " "
                                << byteToHex(0x02) << (rxBuff[3] != static_cast<char>(0x02)) << " "
                                << byteToHex(pro_address.asByte[3]) << (rxBuff[4] != static_cast<char>(pro_address.asByte[3])) << " "
                                << byteToHex(pro_address.asByte[2]) << (rxBuff[5] != static_cast<char>(pro_address.asByte[2])) << " "
                                << byteToHex(pro_address.asByte[1]) << (rxBuff[6] != static_cast<char>(pro_address.asByte[1])) << " "
                                << byteToHex(pro_address.asByte[0]) << (rxBuff[7] != static_cast<char>(pro_address.asByte[0])) << " ";
                                
                                
                                std::cout << "find profile header : ";
                                for (size_t j = 0; j < 8; j++){
                                    std::cout << byteToHex(rxBuff[j]) << " ";
                                }
                                std::cout << " : end" << std::endl;
                                rxBuff.erase(0, 1);

                    }else{
                        std::cerr << "find header" << std::endl;




                        if (type_tag !=10){ // 10 For Profile
                            std::cerr << "for type_tag !=10 : " << type_tag << std::endl;

                            int length = rxBuff[1]*2; // length for 1 word = 2 byte
                            std::cerr << "length : " << length << std::endl;

                            if (20 > length && length>=0){ // this ifelse lenght must not bigger than 20

                                std::cerr << "size : " << rxBuff.size() << ", length+6 : " << 6+length << std::endl;

                                if (rxBuff.size() > (6+length)){

                                    print_rxBuff(rxBuff,"finding ETX : ");

                                    if (rxBuff[4+length] == 0x03){

                                        std::stringstream ss_forChecksum;
                                        ss_forChecksum << rxBuff.substr(1, 4+length-1);

                                        // for (int i = 1; i< (4+length);i++){

                                        //     std::cout << "for check sum : " << rxBuff[i] << std::endl;
                                        // }
                                        uint8_t checksum = calculateXorChecksum(ss_forChecksum);

                                        std::cout << " checksum : 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(checksum) << " "<< std::endl;


                                        if (rxBuff[4+length+1] != static_cast<char>(checksum)){ // Checksum not correct
                                            print_rxBuff(rxBuff,"Fail Checksum : ");
                                            rxBuff.erase(0, 1); 

                                        }else{ 
                                            // Checksum Correct
                                            print_rxBuff(rxBuff,"Pass Checksum : ");

                                            // Check for Command
                                            if(type_tag == sent_Trig){
                                                if (rxBuff[2] == static_cast<char>(0xc0) && rxBuff[3]== static_cast<char>(0x05)){
                                                    type_tag = idle;
                                                    
                                                }
                                                
                                            }else if(type_tag == sent_Hole){
                                                if (rxBuff[2] == static_cast<char>(0xa0) && rxBuff[3]== static_cast<char>(0x15)){
                                                    type_tag = idle;
                                                }

                                            }else if(type_tag == sent_getProAddress){
                                                if (rxBuff[2] == static_cast<char>(0x40) && rxBuff[3]== static_cast<char>(0x0B)){

                                                    pro_address.asByte[0] = rxBuff[7];
                                                    pro_address.asByte[1] = rxBuff[6];
                                                    pro_address.asByte[2] = rxBuff[5];
                                                    pro_address.asByte[3] = rxBuff[4];

                                                    type_tag = idle;
                                                }
                                                
                                            }else if(type_tag == sent_getProSize){
                                                if (rxBuff[2] == static_cast<char>(0x00) && rxBuff[3]== static_cast<char>(0x02)){
                                                    
                                                    pro_size.asByte[1] = rxBuff[8];
                                                    pro_size.asByte[0] = rxBuff[9];

                                                    type_tag = idle;
                                                }
                                            }else if(type_tag == get_Trig){
                                                std::cout <<" in get_Trig " << std::endl;
                                                std::cout <<" con1 " << (rxBuff[2] == static_cast<char>(0xc0)) << std::endl;
                                                std::cout <<" con2 " << (rxBuff[3] == static_cast<char>(0x06)) << std::endl;


                                                if (rxBuff[2] == static_cast<char>(0xc0) && rxBuff[3]== static_cast<char>(0x06)){
                                                    std::cout <<" get_Trig " << std::endl;
                                                    
                                                    tag_reply.asByte[0] = rxBuff[4];
                                                    tag_reply.asByte[1] = rxBuff[5];

                                                    type_tag = idle;
                                                }
                                            }

                                            std::cout <<" type_tag : " << type_tag<< std::endl;
                                            print_rxBuff(rxBuff,"complete : ");
                                            rxBuff.erase(0, 1); // if  clear first one after print
                                            
                                        }

                                    }else{

                                        // print_rxBuff(rxBuff,"Lack : ");
                                        rxBuff.erase(0, 1); // if not have 0x03, ETX end clear first one
                                    }
                                }
                            }else{
                                if (length <0){
                                    std::cout <<" length < 0 " << std::endl;

                                }else{
                                    std::cout <<" length > 20 " << std::endl;
                                }
                                rxBuff.erase(0, 1);
                            }
                        }else{
                            // depackage for profile
                            std::cout <<" get Profile depackage : ";
                            for (size_t j = 0; j < 8; j++){
                                std::cout << byteToHex(rxBuff[j]) << " ";
                            }
                            std::cout << " : end" << std::endl;


                            // uint16_t length = (pro_size.asUInt/32)*2; // length for 1 word = 2 byte
                            uint8_t length_raw = static_cast<char>(rxBuff[1]); // length for 1 word = 2 byte
                            int length = length_raw*2;
                            std::cerr << "length : " << int(length) << std::endl;
                            std::cerr << "get Profile : " << int(length)<<"/"<< rxBuff.size() << std::endl;
                            // print_rxBuff(rxBuff,"get Profile : ");

                            


                            if (rxBuff.size() >= (6+length)){

                                
                                if (rxBuff[4+length] == 0x03){ //

                                    std::stringstream ss_forChecksum;
                                    ss_forChecksum << rxBuff.substr(1, 4+length-1);
                                    // for (int i = 1; i< (4+length);i++){
                                    //     std::cout << "for check sum : " << rxBuff[i] << std::endl;
                                    // }
                                    uint8_t checksum = calculateXorChecksum(ss_forChecksum);
                                    std::cout << " checksum : 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(checksum) << " "<< std::endl;


                                    if (rxBuff[4+length+1] != static_cast<char>(checksum)){ // Checksum not correct
                                            // print_rxBuff(rxBuff,"Fail Checksum : ");
                                            std::cout << " Fail Checksum Profile "<< std::endl;

                                            rxBuff.erase(0, 1); 

                                    }else{ 
                                        // Checksum Correct
                                        // print_rxBuff(rxBuff,"Pass Checksum : ");
                                        std::cout << " Pass Checksum Profile "<< std::endl;

                                        // Checksum Correct

                                        for(int i = 0; i<(length/4);i++){
                                            profile_data[i].x.asByte[1] = rxBuff[8+(i*4)];
                                            profile_data[i].x.asByte[0] = rxBuff[9+(i*4)];
                                            profile_data[i].y.asByte[1] = rxBuff[10+(i*4)];
                                            profile_data[i].y.asByte[0] = rxBuff[11+(i*4)];
                                        }
                                        
                                        type_tag = idle;
                                        std::cout << " Complete Profile "<< std::endl;

                                        // print_rxBuff(rxBuff,"complete : ");
                                        rxBuff.erase(0, 1); // if  clear first one after print
                                    }

                                }else{

                                    std::cout << "checksum require : "<< 6+length << std::endl;
                                    std::cout << "checksum print : "<< rxBuff.size() << std::endl;
                                    std::cout << "Lack : ";
                                    for (size_t j = 0; j < 10+length; j++){
                                        std::cout << byteToHex(rxBuff[j]) << " ";
                                    }
                                    std::cout << " : end" << std::endl;

                                    // print_rxBuff(rxBuff,"Lack : ");
                                    // std::cout << "ETX +- : "
                                    //     << byteToHex(rxBuff[4+length-5]) << " "
                                    //     << byteToHex(rxBuff[4+length-4]) << " "
                                    //     << byteToHex(rxBuff[4+length-3]) << " "
                                    //     << byteToHex(rxBuff[4+length-2]) << " "
                                    //     << byteToHex(rxBuff[4+length-1]) << " m"
                                    //     << byteToHex(rxBuff[4+length]) << "m "
                                    //     << byteToHex(rxBuff[4+length+1]) << " "
                                    //     << byteToHex(rxBuff[4+length+2]) << " "
                                    //     << byteToHex(rxBuff[4+length+3]) << " "
                                    //     << byteToHex(rxBuff[4+length+4]) << " "
                                    //     << byteToHex(rxBuff[4+length+5]) << " "
                                    //     << std::endl;

                                    std::cout << " Lack ETX 0x03 : 0x"<< std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(rxBuff[4+length]) << " " << std::endl;

                                    rxBuff.erase(0, 1); // if not have 0x03, ETX end clear first one
                                }
                            }
                            

                        }


                    }

                    if (type_tag == idle){
                        break;
                    }
                }

            }
        }else{
            std::cout << " Clear Data " << std::endl;

            for (size_t i = 0; i < rxBuff.size(); i++){
                rxBuff.erase(0, 1);
            }
        }

        if (type_tag ==0){

            std::cout << " Clear Data " << std::endl;
            std::cout << "rxBuff size: "<< std::dec << rxBuff.size() << std::endl;

            for (size_t i = 0; i < rxBuff.size(); i++){
                rxBuff.erase(0, 1);
            }
            std::cout << "rxBuff size: "<< std::dec << rxBuff.size() << std::endl;

        }

    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fastus_serial");
    ros::NodeHandle nh;
    FastusProxy fastusProxy("/dev/ttyUSB0", 115200, &nh);
	ros::AsyncSpinner spinner(0);
	spinner.start(); // spin();
    fastusProxy.run();
	ros::waitForShutdown();

    return 0;
}
