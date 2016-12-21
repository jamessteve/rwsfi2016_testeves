/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>

/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */
using namespace std;
using namespace ros;


/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */
class MyPlayer: public rwsfi2016_libs::Player
{
  public: 

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){};

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
      //Custom play behaviour. Now I will win the game

      //Behaviour follow the closest prey
	double dist_min = 1000;
	int angleMin = 0;
	double dist = 0;
	for (int pl=0; pl<preys_team->players.size(); pl++) {
		dist = getDistanceToPlayer(preys_team->players[pl]);
	if (dist_min < dist) {
		angleMin = pl;
		dist_min = dist;
	}
	}
double dist_min_hunter = 1000;
double dist_hunter = 0;
int angleMinHunter = 0;
	for (int pl=0; pl<hunters_team->players.size(); pl++) {
		dist_hunter = getDistanceToPlayer(hunters_team->players[pl]);
	if (dist_min_hunter < dist_hunter) {
		angleMinHunter = pl;
		dist_min_hunter = dist_min;
	}
	}
if (dist_min_hunter < dist_min) {
move(msg.max_displacement, getAngleToPLayer(hunters_team->players[angleMinHunter])-M_PI);
} else {
      move(msg.max_displacement, getAngleToPLayer(preys_team->players[angleMin]));
}
    }
};


/**
 * @brief The main function. All you need to do here is enter your name and your pets name
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 * @return result
 */
int main(int argc, char** argv)
{
  // ------------------------
  //Replace this with your name
  // ------------------------
  string my_name = "testeves";
  string my_pet = "/cheetah";

  //initialize ROS stuff
  ros::init(argc, argv, my_name);

  //Creating an instance of class MyPlayer
  MyPlayer my_player(my_name, my_pet);

  //Infinite spinning (until ctrl-c)
  ros::spin();
}
