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
    int fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY | O_NDELAY);
    if(fd == -1) {
        std::cout << "nopez" << std::endl;
        return 0;
    }
    std::cout << "Fil öppnad" << std::endl;
    if(tcgetattr(fd, &cfg) < 0) {
        std::cout << "Kunde inte hämta konfigurationen" << std::endl;
        close(fd);
        return 0;
    }
    std::cout << "Hämtade konfigurationen" << std::endl;

    if(tcsetattr(fd, TCSAFLUSH, &cfg) < 0) {
        std::cout << "Konfigurationen ej applicerad?" << std::endl;
        close(fd);
        return 0;
    }
    std::cout << "Konfigurationen applicerad?" << std::endl;

    char byte;

    ros::init(argc, argv, "taglistener");
    ros::NodeHandle n;
    tag_listener_pub = n.advertise<position_estimation::Anchor_msgs>("/anchor_dist", 10);
    position_estimation::Anchor_msgs msg;
    msg.anc0 = -1;
    msg.anc1 = -1;
    msg.anc2 = -1;
    msg.anc3 = -1;

    char prevAnchor = -1;
    char stopAnchor = 3;
    char polled_mask = 0x00;
    while(ros::ok()) {
        short anchor;
        bool publishAfterRead = false;
        char mask = 0x00;
        unsigned short distance;
        while(read(fd, &byte, 1) <= 0 || byte != 'A') {}
        while(read(fd, &byte, 1) <= 0) {}

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
            std::cout << "Skumt fel, avbryter... (" << byte << ")" << std::endl;
            close(fd);
            return 0;
        }
        if(polled_mask & mask > 0) {
            publishAfterRead = true;
        }

        // Läs avstånd
        std::string s("");
        while(read(fd, &byte, 1) <= 0) {}

        while(byte != 'm') {
            s = s + byte;
            while(read(fd, &byte, 1) <= 0) {}
        }

        try {
            distance = boost::lexical_cast<unsigned short>(s);
        }

        catch(const boost::bad_lexical_cast &) {
            std::cout << "Skumt fel igen, avbryter... (" << s << ")" << std::endl;
            close(fd);
            return 0;
        }
        std::cout << "Avstånd till ankare " << anchor << ": " << distance << std::endl;
        if(prevAnchor >= anchor) {
            tag_listener_pub.publish(msg);
            msg.anc0 = -1;
            msg.anc1 = -1;
            msg.anc2 = -1;
            msg.anc3 = -1;
            polled_mask = 0x00;
        }
        switch (anchor) {
            case 0:
                msg.anc0 = distance;
                break;
            case 1:
                msg.anc1 = distance;
                break;
            case 2:
                msg.anc2 = distance;
                break;
            case 3:
                msg.anc3 = distance;
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
