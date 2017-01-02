#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "vision/ChessBoard.h"
#include "vision/ChessboardSquare.h"
#include <stdlib.h> 
#include <std_msgs/String.h>
#include <iostream>   // std::cout
#include <string>     // std::string, std::stoi


/*string colour(script,clr){
    #clr
    #30black
    #31red
    #32green
    #33yellow
    #34blue
    #35purple
    #36blue
    #37default white
    return "\033["+str(clr)+"m" + script + "\033[0m";
}*/

vision::ChessBoard init_chessBoard(vision::ChessBoard chessboard){
	vision::ChessboardSquare square;
	square.piece_color="empty";
	square.category="empty";
	square.piece_height=0;

	int column = 0;
	for(int x=0;x<64;x++){
		if(x%8==0&&x!=0){
			column++;
		}
		if ((x+column)%2==0){
			square.square_color="black";
		}else square.square_color="white";

		chessboard.chessSquare.push_back(square);	
	}
	return chessboard;
}

vision::ChessBoard init_piecesPositions(vision::ChessBoard chessboard){
	vision::ChessboardSquare square;
	
	int column = 0;
	int x=0;
	for(int j=0;j<8;j++){ //columns 
		for(int i=0;i<8;i++){ //lines
			if(i==1){
				square.category="pawn";
				square.piece_color="white";
				square.piece_height=4; //4 cm ..
			}else if(i==6){
				square.category="pawn";
				square.piece_height=4;
				square.piece_color="black";				
			}else if(i==0){				
				square.piece_color="white";
				if(j==0||j==7){
					square.category="tower";
					square.piece_height=4.4;
				}else if(j==1||j==6){
					square.category="knight";
					square.piece_height=5.5;
				}else if(j==2||j==5){
					square.category="bishop";
					square.piece_height=5.4;					
				}else if(j==3){
					square.category="queen";
					square.piece_height=7.8;					
				}else if(j==4){
					square.category="king";
					square.piece_height=7.9;					
				}
			}else if(i==7){
				square.piece_color="black";
				if(j==0||j==7){
					square.category="tower";
					square.piece_height=4.4;
				}else if(j==1||j==6){
					square.category="knight";
					square.piece_height=5.5;
				}else if(j==2||j==5){
					square.category="bishop";
					square.piece_height=5.4;					
				}else if(j==3){
					square.category="king";
					square.piece_height=7.9;					
				}else if(j==4){
					square.category="queen";
					square.piece_height=7.8;					
				}
			}else{
				square.piece_color="empty";
				square.piece_height=0;
				square.category="empty";
			}

			if(x%8==0&&x!=0){
				column++;
			}
			if ((x+column)%2==0){
				square.square_color="black";
			}else square.square_color="white";

			chessboard.chessSquare[x].piece_color=square.piece_color;
			chessboard.chessSquare[x].category=square.category;
			chessboard.chessSquare[x].square_color=square.square_color;
			chessboard.chessSquare[x].piece_height=square.piece_height;
			x++;
		}
	}
	return chessboard;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "Game_Menu_node");

  ros::NodeHandle n("~");
  ros::Publisher chessboard_pub = n.advertise<vision::ChessBoard>("chessboard_state", 10);
  //ros::Rate loop_rate(10);


	vision::ChessBoard chessboard;
	chessboard=init_chessBoard(chessboard);

	/*  	for(int i=0;i<chessboard.chessSquare.size();i++){
		ROS_INFO("chessboard i=%d color %s",i,chessboard.chessSquare[i].square_color.c_str());
	}*/
	std::string selection="0";bool terminate=false;
	while(selection.compare("0")==0&&!terminate){
		//ROS_INFO("sizing %d",chessboard.chessSquare.size());
		std::cout << "\n # # # # # # # # # # # # # # # # # # # # # # # #\n\
Select one Option from below:\n\n\
*Initialize Game Board\t\t [1]\n\
*Start New Game\t\t\t [2]\n\
*Add Piece on the Chessboard\t [3]\n\
*Print Chessboard\t\t [4]\n\n\
* Exit [0]\n\n>>";
		std::getline(std::cin, selection);
		if (selection.compare("1")==0){			
			chessboard.chessSquare.clear();
			chessboard=init_chessBoard(chessboard);
			std::cout << "Initialization done..\n";
		}else if(selection.compare("2")==0){
			chessboard=init_piecesPositions(chessboard);
			std::cout << "Pieces repositioned to their initial positions..\n";
		}else if(selection.compare("3")==0){
			
			vision::ChessboardSquare square;
			std::string piece_choice="0";
			int line,column,square_id;
			//std::string::size_type sz; 
			
			std::cout << "# # # # # # # # # # # # # # # # # # # # # # # #\n\nPlease insert the Piece's information.\n(0:cancel)*Line: ";
			std::getline(std::cin, piece_choice);
			if(piece_choice.compare("0")!=0){
				line = atoi(piece_choice.c_str());
				//line = std::stoi (piece_choice,&sz);
				std::cout << "\n(0:cancel)*Column: ";

				std::getline(std::cin, piece_choice);
				if(piece_choice.compare("0")!=0){

					column = atoi(piece_choice.c_str());
					//column = std::stoi (piece_choice,&sz);

					square_id=(column*8-1)-(8-line);
					square.square_color = chessboard.chessSquare[square_id].square_color;

					std::cout << "\n(0:cancel)*Piece color: ";
					std::getline(std::cin, piece_choice);
					if(piece_choice.compare("0")!=0&&(piece_choice.compare("black")==0||piece_choice.compare("white")==0)){
						
						square.piece_color = piece_choice;
						
						std::cout << "\n(0:cancel)*Piece category: ";
						std::getline(std::cin, piece_choice);
						if(piece_choice.compare("0")!=0&&(piece_choice.compare("pawn")==0||piece_choice.compare("knight")==0||piece_choice.compare("bishop")==0||piece_choice.compare("tower")==0||piece_choice.compare("king")==0||piece_choice.compare("queen")==0)){
							square.category = piece_choice;
							if(piece_choice.compare("pawn")==0){
								square.piece_height=4;
							}else if(piece_choice.compare("knight")==0){
								square.piece_height=5.5;
							}else if(piece_choice.compare("bishop")==0){
								square.piece_height=5.4;		
							}else if(piece_choice.compare("tower")==0){
								square.piece_height=4.4;
							}else if(piece_choice.compare("king")==0){
								square.piece_height=7.9;	
							}else if(piece_choice.compare("queen")==0){
								square.piece_height=7.8;	
							}
							//success
							std::cout << "\n\ninsertion done!";
							chessboard.chessSquare[square_id].square_color=square.square_color;
							chessboard.chessSquare[square_id].category=square.category;
							chessboard.chessSquare[square_id].piece_height=square.piece_height;
							chessboard.chessSquare[square_id].piece_color=square.piece_color;

						}
					}
				}
			}


		}else if (selection.compare("0")==0){
			terminate=true;
		}
		if(selection.compare("4")==0){
			std::string draw_choice="0";
			std::cout << "# # # # # # # # # # # # # # # # # # # # # # # #\n\nChoices : \n*Chess squares Colors [1] \n*Chess pieces Colors [2] \n*Chess pieces categories [3] \n*Chess pieces' heights [4]\n : ";
			std::getline(std::cin, draw_choice);
			if(draw_choice.compare("1")==0){
				for (int i = 7; i>=0; i--)
				{
					std::cout << chessboard.chessSquare[i].square_color << "|" << chessboard.chessSquare[i+8].square_color << "|" << chessboard.chessSquare[i+16].square_color << "|" << chessboard.chessSquare[i+24].square_color << "|" << chessboard.chessSquare[i+32].square_color << "|" << chessboard.chessSquare[i+40].square_color << "|" << chessboard.chessSquare[i+48].square_color<< "|" << chessboard.chessSquare[i+56].square_color <<  "\n";
				}
			}else if(draw_choice.compare("2")==0){
				for (int i = 7; i>=0; i--)
				{
					std::cout << chessboard.chessSquare[i].piece_color << "|" << chessboard.chessSquare[i+8].piece_color << "|" << chessboard.chessSquare[i+16].piece_color << "|" << chessboard.chessSquare[i+24].piece_color << "|" << chessboard.chessSquare[i+32].piece_color << "|" << chessboard.chessSquare[i+40].piece_color << "|" << chessboard.chessSquare[i+48].piece_color<< "|" << chessboard.chessSquare[i+56].piece_color <<  "\n";
				}
			}else if(draw_choice.compare("3")==0){
				for (int i = 7; i>=0; i--)
				{
					std::cout << chessboard.chessSquare[i].category << "|" << chessboard.chessSquare[i+8].category << "|" << chessboard.chessSquare[i+16].category << "|" << chessboard.chessSquare[i+24].category << "|" << chessboard.chessSquare[i+32].category << "|" << chessboard.chessSquare[i+40].category << "|" << chessboard.chessSquare[i+48].category<< "|" << chessboard.chessSquare[i+56].category <<  "\n";
				}
			}else if(draw_choice.compare("4")==0){
				for (int i = 7; i>=0; i--)
				{
					std::cout << chessboard.chessSquare[i].piece_height << "|" << chessboard.chessSquare[i+8].piece_height << "|" << chessboard.chessSquare[i+16].piece_height << "|" << chessboard.chessSquare[i+24].piece_height << "|" << chessboard.chessSquare[i+32].piece_height << "|" << chessboard.chessSquare[i+40].piece_height << "|" << chessboard.chessSquare[i+48].piece_height<< "|" << chessboard.chessSquare[i+56].piece_height <<  "\n";
				}
			}

			std::cout << "\n# # # # # # # # # # # # # # # # # # # # # # # #\n";
		}else system("clear");		
		selection="0";
		chessboard_pub.publish(chessboard);
	}
	std::cout << "Quiting now..\n";
	chessboard.chessSquare.clear();
	ros::spinOnce();
  	return 0;
}
