#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <position_estimation/Anchor_msgs.h>

#define A0_T1 0x01
#define A1_T1 0x11 
#define A2_T1 0x21 
#define A3_T1 0x31 
#define A0_T2 0x02
#define A1_T2 0x12 
#define A2_T2 0x22 
#define A3_T2 0x32 

struct termios cfg;

ros::Publisher tag_listener_pub;

bool waitForSync = true;

int main(int argc, char** argv) {

    /* temporary to check how many anchors are received */
    int data1=0, data2=-1, count=0;

    int fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY | O_NDELAY);
    if(fd == -1) {
        std::cout << "nopez" << std::endl;
        return 0;
    }
    std::cout << "File Opened" << std::endl;
    if(tcgetattr(fd, &cfg) < 0) {
        std::cout << "Could not get configuration" << std::endl;
        close(fd);
        return 0;
    }
    std::cout << "Download configuration" << std::endl;

    if(tcsetattr(fd, TCSAFLUSH, &cfg) < 0) {
        std::cout << "configuration not published??" << std::endl;
        close(fd);
        return 0;
    }
    std::cout << "configuration applied?" << std::endl;

    char byte;
    char byte0,byte1;

    ros::init(argc, argv, "taglistener");
    ros::NodeHandle n;
    tag_listener_pub = n.advertise<position_estimation::Anchor_msgs>("/anchor_dist", 10);
    position_estimation::Anchor_msgs msg;
    msg.anc0_t1 = -1;
    msg.anc1_t1 = -1;
    msg.anc2_t1 = -1;
    msg.anc3_t1 = -1;
    msg.anc0_t2 = -1;
    msg.anc1_t2 = -1;
    msg.anc2_t2 = -1;
    msg.anc3_t2 = -1;

    char prevAnchor = -1;
    char stopAnchor = 3;
    char polled_mask = 0x00;
    while(ros::ok()) {
        short anchor, tag, anchor_tag_value;
        bool publishAfterRead = false;
        char mask = 0x00;
        unsigned short distance;
        std::string in("");

        // Parse A#->T# and extract tag and anchor number 
        while(read(fd, &byte, 1) <= 0 || byte != 'A') {
             //std::cout << byte << "  wait 1"<< std::endl; //MyCode
        }
        //Wait for Anchor message to be received
        while(read(fd, &byte0, 1) <= 0) {
             //std::cout << byte0  <<"  wait 2"<< std::endl; //Mycode
        }
        while(read(fd, &byte1, 1) <= 0) {
             //std::cout << byte1  <<"  wait 2.1"<< std::endl; //Mycode
        }

        // Vänta på sync
        if(waitForSync) {
            if((byte0 == '0') && (byte1 == '1')) {
                waitForSync = false;
                std::cout <<"wait for sync done"<< std::endl; //Mycode
            }
            else {
                continue;
            }
        }
        // convert anchor value to short from char
        try {
            anchor = boost::lexical_cast<unsigned short>(byte0);
            //std::cout<< distance << std::endl;
        }

        catch(const boost::bad_lexical_cast &) {
            std::cout << "Wrong Interrupts !! (" << byte0 << ")" << std::endl;
            close(fd);
            return 0;
        }
        // convert tag value to short from char
        try {
            tag = boost::lexical_cast<unsigned short>(byte1);
            //std::cout<< distance << std::endl;
        }

        catch(const boost::bad_lexical_cast &) {
            std::cout << "Wrong Interrupts !! (" << byte1 << ")" << std::endl;
            close(fd);
            return 0;
        }
        //std::cout << "anchor " << anchor << "tag " <<tag<< std::endl;
        /* Anchor_Tag*/
        anchor_tag_value = ((anchor << 4)|tag);

        //Capture the new byte
        while(read(fd, &byte, 1) <= 0) {}
        // Capture the next bytes indicating distance
        std::string s("");
        while(byte != 'm') {
            if (byte != ' '){ // Do not fill in the space collected.
                s = s + byte;
            }
            //std::cout << byte << "  wait 4"<<std::endl; //Mycode
            while(read(fd, &byte, 1) <= 0) {}
        }
        //std::cout << s << std::endl;

        try {
            distance = boost::lexical_cast<unsigned short>(s);
            //std::cout<< distance << std::endl;
        }

        catch(const boost::bad_lexical_cast &) {
            std::cout << "Conversion of distance to short from char failed !! (" << s << ")" << std::endl;
            close(fd);
            return 0;
        }

        //std::cout << "Distance info anchor_tag_" << anchor_tag_value << ": " << distance << std::endl;

        //Mechanism to send out data when a set of data is received
        if(tag == 1){
            mask = 1<<anchor; // Lower Nibble indicates status of data received from tag 1
        }
        else{
            mask = 1<<(anchor + 4); // the high nibble indicates the status of data received from Tag 2
        }

        if(polled_mask & mask > 0) {
            tag_listener_pub.publish(msg);
            msg.anc0_t1 = -1;
            msg.anc1_t1 = -1;
            msg.anc2_t1 = -1;
            msg.anc3_t1 = -1;
            msg.anc0_t2 = -1;
            msg.anc1_t2 = -1;
            msg.anc2_t2 = -1;
            msg.anc3_t2 = -1;
            polled_mask = 0x00;
        }
        switch (anchor_tag_value) {
            case A0_T1:
                msg.anc0_t1 = distance;
                break;
            case A1_T1:
                msg.anc1_t1 = distance;
                break;
            case A2_T1:
                msg.anc2_t1 = distance;
                break;
            case A3_T1:
                msg.anc3_t1 = distance;
                break;
            case A0_T2:
                msg.anc0_t2 = distance;
                break;
            case A1_T2:
                msg.anc1_t2 = distance;
                break;
            case A2_T2:
                msg.anc2_t2 = distance;
                break;
            case A3_T2:
                msg.anc3_t2 = distance;
                break;
            default:
            break;
        }
        polled_mask = polled_mask | mask;
    }
    close(fd);
    return 0;
}
