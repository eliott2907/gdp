#!/usr/bin/env python3

'''This code is part of the GDP project of team 2 from the Robotics Msc 2023-2024 Cranfield University
The goal of this code is to automaticaly send an email that contains an anomaly type, a timestemp and a google maps of the location of the robot.
This function is a standalone function that was created mainly for testing if it would be possible to create a google maps link based on the position of the robot but it was not integrated in our full 
communication system. The sender email function used in the system is also available in the Git repo or in the appendix of the

Written by Eliott Laurent April 2024
Contact: eliott.laurent.955@cranfield.ac.uk'''


import rospy
import smtplib
import math
from geometry_msgs.msg import Pose
from geopy.geocoders import Nominatim
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from datetime import datetime
from std_msgs.msg import Int32
import pytz

#Declare global variables
anom=None #tempoarly stores anomaly types
count=0 #counts the number of anomalies


#This function sends an email taking as an input the anomaly type and a google maps link
def send_email(google_maps_link,anom):

    # Email details
    sender_email = 'eliott.laurent@ecam.fr'
    sender_password = ''#insert email password here
    recipient_email = 'eliott.laurent1@gmail.com'
    

    # Get current time in UK timezone
    uk_timezone = pytz.timezone('Europe/London')
    current_time = datetime.now(uk_timezone).strftime('%Y-%m-%dT%H:%M:%S%z')

 
    # Creates the text that will be sent in the email
    if anom ==1:
        anom_type= 'Fire was detected!'
    subject = anom_type
    message = f'{anom_type}\nThis is the location of the anomaly:{google_maps_link}\n Timestamp:{current_time}'

    # Create message container
    msg = MIMEMultipart()
    msg['From'] = sender_email
    msg['To'] = recipient_email
    msg['Subject'] = subject

    # Attach message
    msg.attach(MIMEText(message, 'plain'))

    # Connect to SMTP server (Gmail)
    server = smtplib.SMTP('smtp.ecam.fr', 587) #these parameters will change depending on the email adress used for sending the email
    server.starttls()
    server.login(sender_email, sender_password)

    # Send email
    text = msg.as_string()
    server.sendmail(sender_email, recipient_email, text)

    # Close the SMTP server connection
    server.quit()

#this functiion takes as an input lattitude and longitude and oututs a google maps link
def get_google_maps_link(latitude, longitude):
    return f"https://www.google.com/maps?q={latitude},{longitude}"


#this function takes as an input the x and y coordinates on the gmap into lattitude and longitude
def convert_pose(x,y):
    #declares the origin position and the orientation of the reference frame of the gmap
    origin_latitude= 52.07119515689871
    origin_longitude= -0.628428774094947
    theta=0 #in radians

    #computes x and y that are colinear to the latitude and longitude info
    newX=x*math.cos(theta)-y*math.sin(theta)
    newY=x*math.sin(theta)-y*math.cos(theta)

    #Converts the x and y coordinates into lattitude and longitude
    new_latitude= origin_latitude + (newY/111111)
    new_longitude= (newX/(111111*math.cos(origin_latitude*(math.pi/180))))+origin_longitude
    google_maps_link = get_google_maps_link(new_latitude, new_longitude)

    #sends email
    send_email(google_maps_link,anom)


# Callback function
def anomaly_callback(data):
    global anom
    global count
    #if anomaly is detected its type is stored
    if isinstance(data, Int32):
        anom=data.data
    
    #if the position is published and if the email was not already sent the email is sent
    if isinstance(data, Pose):
        x=data.position.x
        y=data.position.y
        if count ==0:
            if anom==1:
                convert_pose(x,y)
                count=count+1
            

# Main function that subscribes to the robot position and the anomaly topics          
if __name__ == '__main__':
    rospy.init_node("latitude_longitude")
    rospy.Subscriber("/anomaly",Int32,anomaly_callback)
    rospy.Subscriber("/robot_pose",Pose,anomaly_callback)
    rospy.spin()
