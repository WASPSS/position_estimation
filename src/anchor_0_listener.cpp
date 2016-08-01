#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <position_estimation/Anchor_msgs.h>

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

    ros::init(argc, argv, "taglistener");
    ros::NodeHandle n;
    tag_listener_pub = n.advertise<position_estimation::Anchor_msgs>("/anchor_dist", 10);
    position_estimation::Anchor_msgs msg;
    msg.anc0_t0 = -1;
    msg.anc1_t0 = -1;
    msg.anc2_t0 = -1;
    msg.anc3_t0 = -1;
    msg.anc0_t1 = -1;
    msg.anc1_t1 = -1;
    msg.anc2_t1 = -1;
    msg.anc3_t1 = -1;

    char prevAnchor = -1;
    char stopAnchor = 3;
    char polled_mask = 0x00;
    while(ros::ok()) {
        short anchor;
        bool publishAfterRead = false;
        char mask = 0x00;
        unsigned short distance;
        while(read(fd, &byte, 1) <= 0 || byte != 'A') {
        	 //std::cout << byte << "  wait 1"<< std::endl; //MyCode
        }
        //Wait for Anchor message to be received
        while(read(fd, &byte, 1) <= 0) {
        	 //std::cout << byte  <<"  wait 2"<< std::endl; //Mycode
        }

        // Vänta på sync
        if(waitForSync) {
            if(byte == '0') {
                waitForSync = false;
            }
            else {
                continue;
            }
        }
        switch(byte) {
        case '0':
            anchor = 0;
            mask = 0x01;
            break;
        case '1':
            anchor = 1;
            mask = 0x02;
            break;
        case '2':
            anchor = 2;
            mask = 0x04;
            break;
        case '3':
            anchor = 3;
            mask = 0x08;
            break;
        default:
            std::cout << "Fishy Errors / Interrupts (" << byte << ")" << std::endl;
            close(fd);
            return 0;
        }

        /* Logic to output count */

        data1 = anchor;
        if(data1 > data2){ //Old set of anchors are read
            count++;
        }
        else{
            std::cout << count << std::endl;
            count = 1;
        }
        data2 = data1;

        if(polled_mask & mask > 0) {
            publishAfterRead = true;
        }

        // Läs avstånd
        std::string s("");
        while((read(fd, &byte, 1) <= 0) || (byte != ' ')) {
        	 //std::cout << byte  << "  wait 3"<<std::endl; //Mycode
        }
        //std::cout<< " stopped at space " << std::endl;

        while(byte != 'm') {
            if (byte != ' '){ // Do not fill in the empty string collected in above while.
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
            std::cout << "Wrong Interrupts !! (" << s << ")" << std::endl;
            close(fd);
            return 0;
        }
        std::cout << "Avstånd till ankare " << anchor << ": " << distance << std::endl;
        if(prevAnchor >= anchor) {
            tag_listener_pub.publish(msg);
            msg.anc0_t0 = -1;
            msg.anc1_t0 = -1;
            msg.anc2_t0 = -1;
            msg.anc3_t0 = -1;
            msg.anc0_t1 = -1;
            msg.anc1_t1 = -1;
            msg.anc2_t1 = -1;
            msg.anc3_t1 = -1;
            polled_mask = 0x00;
        }
        switch (anchor) {
            case 0:
                msg.anc0_t0 = distance;
                break;
            case 1:
                msg.anc1_t0 = distance;
                break;
            case 2:
                msg.anc2_t0 = distance;
                break;
            case 3:
                msg.anc3_t0 = distance;
                break;
            default:
            break;
        }
        prevAnchor = anchor;
        polled_mask = polled_mask | mask;
    }
    close(fd);
    return 0;
}
