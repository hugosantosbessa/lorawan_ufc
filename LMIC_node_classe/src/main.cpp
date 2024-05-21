#include "LMIC-node.h"

uint16_t counterValue = 1;
char data[20] = "";
LMICNode node;


//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀

void prepareUplink(){
    // This function must be defined by the user. Define the date it will be sent.

    // Use globals: LMICNode::fPort, LMICNode::mydata, LMICNode::sizeData
    
    // This is where the main work is performed like
    // reading sensor and GPS data and schedule uplink
    // messages if anything needs to be transmitted.

    strcpy((char*)LMICNode::mydata, "Hello, World! Message number ");
    snprintf(data, sizeof(data), "%d!", counterValue++);
    strcat((char*)LMICNode::mydata, data);
    LMICNode::sizeData = strlen((char*)LMICNode::mydata);
    LMICNode::printSpaces(BSF::serial, MESSAGE_INDENT);
    BSF::serial.print("Mydata: ");
    BSF::serial.println((char*)LMICNode::mydata);
    LMICNode::printSpaces(BSF::serial, MESSAGE_INDENT);
    BSF::serial.print("Sizedata: ");
    BSF::serial.println(LMICNode::sizeData);
    LMICNode::fPort = 10;
    LMICNode::scheduleUplink(LMICNode::fPort, LMICNode::mydata, LMICNode::sizeData);
}

//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀ 


void setup() {
    node.initHardware();

    // Setting the data rate is optional. 
    // By default, it is set to DR_SF7.
    node.setDataRate(DR_SF10);
    
    // This function makes the initial configurations of LMIC. 
    // Then, all the configurations you need must be defined beforehand.
    node.initLmic();  

    //  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
    //  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
    //  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀

    // Place code for initializing sensors etc. here.

    //  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
    //  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
    //  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀ 

    node.initTransmission();
}

void loop() {  
    node.loop_node();
}
