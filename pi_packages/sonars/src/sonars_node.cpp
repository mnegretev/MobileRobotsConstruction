#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "wiringPi.h"

#define PIN_TRIGGER 0  //Header 11
#define PIN_SONAR_0 1  //Header 12
#define PIN_SONAR_1 2  //Header 13
#define PIN_SONAR_2 3  //Header 15
#define PIN_SONAR_3 4  //Header 16
#define PIN_SONAR_4 5  //Header 18
#define PIN_SONAR_5 6  //Header 22
#define PIN_SONAR_6 7  //Header 07
#define PIN_SONAR_7 10 //Header 24

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SONARS NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "sonars");
    ros::NodeHandle n;

    ros::Publisher pub_sonars = n.advertise<std_msgs::Float32MultiArray>("/rotombot/hardware/distance_sensors", 1);
    ros::Rate loop(30);

    std::vector<int>   pin_assignments;
    std::vector<bool>  sonar_measured;
    std_msgs::Float32MultiArray msg_sonars;

    pin_assignments.resize(8);
    msg_sonars.data.resize(pin_assignments.size());
    sonar_measured.resize(pin_assignments.size());

    pin_assignments[0] = PIN_SONAR_4;
    pin_assignments[1] = PIN_SONAR_5;
    pin_assignments[2] = PIN_SONAR_6;
    pin_assignments[3] = PIN_SONAR_7;
    pin_assignments[4] = PIN_SONAR_0;
    pin_assignments[5] = PIN_SONAR_1;
    pin_assignments[6] = PIN_SONAR_2;
    pin_assignments[7] = PIN_SONAR_3;

    int short_wait = 150;
    int long_wait = 8000;
    long start_time;

    wiringPiSetup();
    pinMode(PIN_TRIGGER, OUTPUT);
    for(int i=0; i < pin_assignments.size(); i++)
        pinMode(pin_assignments[i], INPUT);

    digitalWrite(PIN_TRIGGER, LOW);
    delay(30);

    while(ros::ok())
    {
        short_wait = 300;
        long_wait = 8000;

        for(int i=0; i < pin_assignments.size(); i++)
            sonar_measured[i] = false;

        digitalWrite(PIN_TRIGGER, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_TRIGGER, LOW);

        for(int i=0; i < pin_assignments.size(); i++)
            while(digitalRead(pin_assignments[i]) == LOW && --short_wait >= 0)
                delayMicroseconds(5);

        start_time = micros();
        while(--long_wait >= 0)
        {
            for(int i=0; i < pin_assignments.size(); i++)
                if(!sonar_measured[i] && digitalRead(pin_assignments[i]) == LOW)
                {
                    sonar_measured[i] = true;
                    msg_sonars.data[i] = (micros() - start_time)*0.0001715;
                }
            delayMicroseconds(5);
        }

        pub_sonars.publish(msg_sonars);
        ros::spinOnce();
        loop.sleep();
    }
}
