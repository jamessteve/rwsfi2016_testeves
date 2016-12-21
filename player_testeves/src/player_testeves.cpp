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
        //Behaviour follow the closest prey
        double dist_min = 100000;
        int angleMin = 0;
        double dist = 0;
        for (int pl=0; pl < preys_team->players.size(); pl++) {
            dist = getDistanceToPlayer(preys_team->players[pl]);
            if ((dist < dist_min) && (!isnan(dist) ) ) {
                angleMin = pl;
                dist_min = dist;
            }
        }
        double dist_min_hunter = 100000;
        double dist_hunter = 0;
        int angleMinHunter = 0;
        for (int pl=0; pl < hunters_team->players.size(); pl++) {
            dist_hunter = getDistanceToPlayer(hunters_team->players[pl]);
            if ((dist_hunter < dist_min_hunter) && (!isnan(dist_hunter))) {
                angleMinHunter = pl;
                dist_min_hunter = dist_hunter;
            }
        }
        // Find team mates
        double dist_min_team = 100000;
        double dist_team = 0;
        int angleMinteam = 0;
        for (int pl=0; pl < my_team->players.size(); pl++) {
            dist_team = getDistanceToPlayer(my_team->players[pl]);
            if ((dist_team < dist_min_team) && (!isnan(dist_team))) {
                angleMinteam = pl;
                dist_min_team = dist_team;
            }
        }

        double finalAngle = 0.0;
        if (dist_min_hunter < dist_min) {
            if (dist_min_hunter < dist_min_team) {
                ROS_INFO_STREAM("Hunter mais proximo: " << hunters_team->players[angleMinHunter] << " angle: " << getAngleToPLayer(hunters_team->players[angleMin]));
                double angle_temp = getAngleToPLayer(hunters_team->players[angleMinHunter]);
                finalAngle = angle_temp+M_PI;
                if (angle_temp > 0)
                    finalAngle = angle_temp-M_PI;
                //MOVE//
                if (getDistanceToArena() < 7) { // Evaluate if we are moving outside map
                    move(msg.max_displacement, finalAngle);
                } else {
                    move(msg.max_displacement, angle_temp+(M_PI));
                }
            } else {
                ROS_INFO_STREAM("Team mais proximo: " << my_team->players[angleMinHunter] << " angle: " << getAngleToPLayer(my_team->players[angleMin]));
                double angle_temp = getAngleToPLayer(my_team->players[angleMinteam]);
                finalAngle = angle_temp+M_PI;
                if (angle_temp > 0)
                    finalAngle = angle_temp-M_PI;
                //MOVE//
                if (getDistanceToArena() < 7) { // Evaluate if we are moving outside map
                    move(msg.max_displacement, finalAngle);
                } else {
                    move(msg.max_displacement, angle_temp+(M_PI));
                }
            }
        } else {
            ROS_INFO_STREAM("Preyer mais proximo: " << preys_team->players[angleMin] << " angle: " << getAngleToPLayer(preys_team->players[angleMin]));
            //MOVE
            //MOVE//
            if (getDistanceToArena() < 7) { // Evaluate if we are moving outside map
                move(msg.max_displacement, getAngleToPLayer(preys_team->players[angleMin]));
            } else {
                move(msg.max_displacement, getAngleToPLayer(preys_team->players[angleMin])+(M_PI));
            }
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
