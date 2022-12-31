# Exoskeleton Project IITBBS

## Scripts use in ROS

### exo execution with process handler
### exo ros mqtt endpoint multiple joints
### state machine 
### trajectory generator

###  Scripts should be used.

Mqtt end point scripts

1. <ins>**Exo ros mqtt endpoint.**</ins>
    - **Purpose**
        - It acts as the endpoint. Converts the desired ROS Topics into MQTT topics and vice versa.
    - **Working**
        - Uses the struct module to serialize and desrialize the data.
    -
    - **MQTT topics that are converted into ros topics**
        - 
    - **ROS topics that are converted to MQTT topics**
        - 
    - topics graph


### Topics graph


## launch files
    

###

### esp32 setup
    - use this link to install in windows

### setup
    - install the windows version of the esp32 idf.
    - open the terminal in the vscode or command prompt.

    for easy use in vscode
        - create the .worspace file 
        - inside the file enter these
            - (you can make the terminal to run some files on openning the new terminal)
            - for our case we have to load the activate.bat and export.bat from the espidf folder.

    for using in any windows terminal u should do the following

        - run the activate.bat file fro pyenc
        - run the export .bat file
        - This will allow you to you use the idf.py in that terminal
        - navigate to the project folder
        - make sure that the the project is from one of the example folder so that it follows the proper esp32 required cmake elements.

To test the hall sensors.
    - rotate the motor by hand and check if the hall state value are in this order 5,1,3,2,6,4 for anticlockwise and reverse for clockwise.

To test the motor working.
    - using the code if the uvw windings are corrected in right order and the hall sensors values are in right sequence the motor should rotate.

To test the encoders working.
    - check the encoder wiring A and B are connected to the right pins

Testing the mqtt
    - run the mqtt test
    - paho in python


Position and velocity control through paho.
    - mention which file to run.

Trajectory control thruugh python and paho
    - python file.
    - esp32 file.

Trajectory control and position control through ros.
    - packages to use.
    - esp32 files to use.



### <ins>Connections</ins>


