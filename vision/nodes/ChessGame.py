#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic

import rospy
import os
from std_msgs.msg import UInt8, Float32,String
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm
from geometry_msgs.msg import Vector3
import sys,tty,termios
from vision.msg import ChessBoard, ChessboardSquare

########## Menu Controller ########################
# Options___________________                      #
###################################################


class _GetArrowKey:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def init_chessboard(chessboard):
	white_square = ChessboardSquare('empty','empty','black')
	black_square = ChessboardSquare('empty','empty','white')
	column = 0
	for x in range(0, 64):
		print x
		if x%8 == 0 and x!=0:
			column=column+1
		if (x+column)%2 == 0:
			chessboard[x]=white_square
		else:
			chessboard[x]=black_square
		#print square			
	return chessboard

def colour(script,clr):
    #clr
    #30black
    #31red
    #32green
    #33yellow
    #34blue
    #35purple
    #36blue
    #37default white
    return "\033["+str(clr)+"m" + script + "\033[0m"

if __name__ == '__main__':

    rospy.init_node('Game_Menu_node', anonymous=True)

    chessboard_pub = rospy.Publisher("chessboard_state", [ChessboardSquare()  for x in range(64)], queue_size=10)
    msg = rospy.Publisher("user_messages", UInt8, queue_size=10)


    #chessboard=[["" for x in range(2)] for x in range(64)]
    chessboard =[ChessboardSquare()  for x in range(64)];
    init_chessboard(chessboard)

    #import sys
    # for x in range(0, 64):
    # 	if chessboard[x].square_color=='black':
    # 		sys.stdout.write(' . ')
    # 	else:
    # 		sys.stdout.write(' A ')
    # 	if x==7 or x==15 or x==23 or x==31 or x==39 or x==47 or x==55 or x==63 and x!=0:
    # 		sys.stdout.write('\n')
    ##print chessboard
    chessboard_pub.publish(chessboard)
    #os.system("clear")
    selection=0
    #effort_value=0
    while selection==0:

        selection = input(colour("# # # # # # # # # # # # # # # # # # # # # # # #\
 # # # # # # # # # # # # # #",32)+"\n\n\
Select one Option from below:\n\n\
*Initialize Game Board\t\t\t [1]\n\
*" + colour("Start ",35) + colour("New Game",32) +"\t\t [2]\n\
*Add Piece on the Chessboard\t [3]\n\n\
* Exit [0]\n\n>> ")

        
        if selection != 0 and selection<=3 and selection>=1: ##ascii char '5' is equal to #53 and '0' is #48..
        	if selection==1:
        		chessboard = ChessBoard()
        		chessboard_pub.publish(chessboard)
        		#chessboard_pub.publish(chessboard_pub)
 #            if selection==4:
 #                effort_value= input("\nPlease give me a number, in the range [0 to 10], to specify the stiffness\n of the fingers. ( 10 is the most sensitive,0.1 more stiff )\n\nEnter Value> ")
 #                effort_msg.publish(effort_value)
 #            elif selection==10:
 #                print("\nPress the keyboard Arrows (left < and > right) to set the new Robonaut's waist position and (up and\
 # down arrows) to set Robonaut's Arm Start height. (press one another key, to quit)\n")
 #                arrow_value="noth"
 #                while arrow_value!="unknown":
 #                    arrow_value=getArrowForWaistMovement()
                                  
 #            msg.publish(selection)

        else: break
        os.system("clear")
        print (colour("# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #",32)+"\n\n>\tOption '" + str(selection) + "' just published...\n")
        selection=0


    print("Quiting now..\nThe node can be terminated..")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()