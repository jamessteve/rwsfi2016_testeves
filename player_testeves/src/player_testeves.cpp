/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <rwsfi2016_libs/player.h>
#include <rwsfi2016_msgs/GameQuery.h>
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

    ros::Publisher publisher;
    visualization_msgs::Marker bocas_msg;
    // ROSSERVICE
    ros::ServiceServer service;

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){
        publisher = node.advertise<visualization_msgs::Marker>("/bocas", 1);
        bocas_msg.header.frame_id = name;
        bocas_msg.ns = name;
        bocas_msg.id = 0;
        bocas_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        bocas_msg.action = visualization_msgs::Marker::ADD;
        bocas_msg.scale.z = 0.4;
        bocas_msg.pose.position.y = 0.3;
        bocas_msg.color.a = 1.0; // Don't forget to set the alpha!
        bocas_msg.color.r = 0.0;
        bocas_msg.color.g = 0.0;
        bocas_msg.color.b = 0.0;

        service = node.advertiseService("/testeves/game_query", &MyPlayer::queryCallback, this);
    };

    bool queryCallback(rwsfi2016_msgs::GameQuery::Request &req, rwsfi2016_msgs::GameQuery::Response &res)
    {
        res.resposta = "Hello, it is a banana!";
        return true;
    }


    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
        bocas_msg.header.stamp = ros::Time();

        double distance_to_arena = getDistanceToArena();
        ROS_INFO("distance_to_arena = %f", distance_to_arena);

        if (distance_to_arena > 6) { //behaviour move to the center of arena
            string arena = "/map";
            move(msg.max_displacement, getAngleToPLayer(arena));
            bocas_msg.text = "Nao vas para ai pah!!!";
        } else
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
            //            for (int pl=0; pl < hunters_team->players.size(); pl++) {
            //                dist_hunter = getDistanceToPlayer(hunters_team->players[pl]);
            //                if ((dist_hunter < dist_min_hunter) && (!isnan(dist_hunter))) {
            //                    angleMinHunter = pl;
            //                    dist_min_hunter = dist_hunter;
            //                }
            //            }
            double dist_min_team = 100000;
            double dist_team = 0;
            int angleMinteam = 0;
            double finalAngle = 0.0;
            if (dist_min_hunter < dist_min/2) {
                ROS_INFO_STREAM("Hunter mais proximo: " << hunters_team->players[angleMinHunter] << " angle: " << getAngleToPLayer(hunters_team->players[angleMin]));
                double angle_temp = getAngleToPLayer(hunters_team->players[angleMinHunter]);
                finalAngle = angle_temp+M_PI;
                if (angle_temp > 0)
                    finalAngle = angle_temp-M_PI;
                //MOVE//
                move(msg.max_displacement, finalAngle);
                bocas_msg.text = "Foge que vem ai o predador " + hunters_team->players[angleMinHunter];
            } else {
                ROS_INFO_STREAM("Preyer mais proximo: " << preys_team->players[angleMin] << " angle: " << getAngleToPLayer(preys_team->players[angleMin]));
                //MOVE//
                move(msg.max_displacement, getAngleToPLayer(preys_team->players[angleMin]));
                bocas_msg.text = preys_team->players[angleMin] + ", toma toma toma foguetinhos!!!";
            }
        }

        publisher.publish(bocas_msg);

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
    string my_pet = "/cat";

    //initialize ROS stuff
    ros::init(argc, argv, my_name);

    //Creating an instance of class MyPlayer
    MyPlayer my_player(my_name, my_pet);

    //Infinite spinning (until ctrl-c)
    ros::spin();
}
