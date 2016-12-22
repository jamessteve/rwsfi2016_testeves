/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <rwsfi2016_libs/player.h>
#include <rwsfi2016_msgs/GameQuery.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */
using namespace std;
using namespace ros;

typedef pcl::PointXYZRGB PointT;


/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */
class MyPlayer: public rwsfi2016_libs::Player
{
public:

    ros::Publisher publisher;
    ros::Subscriber subscriber;
    pcl::PointCloud<PointT> objectReceived;

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

        // Receive object
        subscriber = node.subscribe("/object_point_cloud", 1, &MyPlayer::receiveObject, this);

    };

    void receiveObject(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // Convert to point cloud
        pcl::fromROSMsg(*msg, objectReceived);
        std::cout << "Object received and converted" << std::endl;
//        std::cout << "Tamanho da point cloud: " << objectReceived.points.size() << std::endl;
    }

    bool queryCallback(rwsfi2016_msgs::GameQuery::Request &req, rwsfi2016_msgs::GameQuery::Response &res)
    {
        // Object to analyze
        double meanValR = 0.0;
        double meanValG = 0.0;
        double meanValB = 0.0;
        int objectSize = 0;
        for (int pt = 0; pt < objectReceived.points.size(); pt++) {
            if (!isnan(objectReceived.points[pt].r)) {
                meanValR += objectReceived.points[pt].r;
                meanValG += objectReceived.points[pt].g;
                meanValB += objectReceived.points[pt].b;
                objectSize++;
            }
        }
        // Compute mean
        meanValR = meanValR/objectSize;
        meanValG = meanValG/objectSize;
        meanValB = meanValB/objectSize;
//        std::cout << "R: " << meanValR << std::endl;
//        std::cout << "G: " << meanValG << std::endl;
//        std::cout << "B: " << meanValB << std::endl;
        if (meanValR > 130) {
            res.resposta = "banana";
        } else {
            if (meanValR > 90) {
                res.resposta = "tomato";
            } else {
                if (meanValB > 70) {
                    res.resposta = "soda_can";
                } else {
                    res.resposta = "onion";
                }
            }
        }

//        if (objectReceived.points.size() == 3805)
//            res.resposta = "soda_can";
//        if (objectReceived.points.size() == 3979)
//            res.resposta = "banana";
//        if (objectReceived.points.size() == 3468)
//            res.resposta = "onion";
//        if (objectReceived.points.size() == 1570)
//            res.resposta = "tomato";

        std::cout << "###############" << std::endl;
        std::cout << "###############" << std::endl;
        std::cout << "Resposta final: " << res.resposta << std::endl;
        std::cout << "###############" << std::endl;
        std::cout << "###############" << std::endl;

        return true;
    }


    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {

        bocas_msg.header.stamp = ros::Time();

        // Player to kill
        int killkill = 0;

        // Distance to arena
        double distance_to_arena = getDistanceToArena();
        if (distance_to_arena > 7.5) { //behaviour move to the center of arena
            string arena = "/map";
            move(msg.max_displacement, getAngleToPLayer(arena));
            bocas_msg.text = "Nao vas para ai pah!!!";
        } else {
            if (msg.blue_alive.size() > 0) { // Se existir algum vivo
                // Kill player id "killkill"
                string kill_player_name = msg.blue_alive.at(killkill);
                for (int pl=0; pl<msg.blue_alive.size(); pl++) {
                    if (kill_player_name.compare(msg.blue_alive.at(pl)) != 0) {
                        // Procura o alive mais perto
                        double dist_min = 100000;
                        int angleMin = 0;
                        double dist = 0;
                        for (int pl=0; pl < msg.blue_alive.size(); pl++) {
                            dist = getDistanceToPlayer(msg.blue_alive.at(pl));
                            if ((dist < dist_min) && (!isnan(dist) ) ) {
                                killkill = pl;
                                dist_min = dist;
                            }
                        }
                    }
                    break;
                }
                move(msg.max_displacement, getAngleToPLayer(msg.blue_alive.at(killkill)));
                bocas_msg.text = msg.blue_alive.at(killkill) + ", toma toma toma foguetinhos!!!";
            } else { // Se estiverem todos mortos
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
                // Foge do que estiver mais perto
                ROS_INFO_STREAM("Hunter mais proximo: " << hunters_team->players[angleMinHunter] << " angle: " << getAngleToPLayer(hunters_team->players[angleMinHunter]));
                double angle_temp = getAngleToPLayer(hunters_team->players[angleMinHunter]);
                double finalAngle = angle_temp+M_PI;
                if (angle_temp > 0)
                    finalAngle = angle_temp-M_PI;
                //MOVE//
                move(msg.max_displacement, finalAngle);
                bocas_msg.text = "Foge que vem ai o predador " + hunters_team->players[angleMinHunter];
            }
        }

        publisher.publish(bocas_msg);


//        bocas_msg.header.stamp = ros::Time();

//        double distance_to_arena = getDistanceToArena();
//        ROS_INFO("distance_to_arena = %f", distance_to_arena);

//        if (distance_to_arena > 7.5) { //behaviour move to the center of arena
//            string arena = "/map";
//            move(msg.max_displacement, getAngleToPLayer(arena));
//            bocas_msg.text = "Nao vas para ai pah!!!";
//        } else
//        {
//            //Behaviour follow the closest prey
//            double dist_min = 100000;
//            int angleMin = 0;
//            double dist = 0;
//            for (int pl=0; pl < preys_team->players.size(); pl++) {
//                dist = getDistanceToPlayer(preys_team->players[pl]);
//                if ((dist < dist_min) && (!isnan(dist) ) ) {
//                    angleMin = pl;
//                    dist_min = dist;
//                }
//            }
//            double dist_min_hunter = 100000;
//            double dist_hunter = 0;
//            int angleMinHunter = 0;
//            for (int pl=0; pl < hunters_team->players.size(); pl++) {
//                dist_hunter = getDistanceToPlayer(hunters_team->players[pl]);
//                if ((dist_hunter < dist_min_hunter) && (!isnan(dist_hunter))) {
//                    angleMinHunter = pl;
//                    dist_min_hunter = dist_hunter;
//                }
//            }
//            //            double dist_min_my = 100000;
//            //            double dist_my = 0;
//            //            int angleMinMy = 0;
//            //            for (int pl=0; pl < my_team->players.size(); pl++) {
//            //                dist_my = getDistanceToPlayer(my_team->players[pl]);
//            //                if ((dist_my < dist_min_my) && (!isnan(dist_my))) {
//            //                    angleMinMy = pl;
//            //                    dist_min_my = dist_my;
//            //                }
//            //            }
//            double finalAngle = 0.0;
//            if (dist_min_hunter < dist_min/2) {
//                //                if (dist_min_hunter < dist_min_my) {
//                ROS_INFO_STREAM("Hunter mais proximo: " << hunters_team->players[angleMinHunter] << " angle: " << getAngleToPLayer(hunters_team->players[angleMin]));
//                double angle_temp = getAngleToPLayer(hunters_team->players[angleMinHunter]);
//                finalAngle = angle_temp+M_PI;
//                if (angle_temp > 0)
//                    finalAngle = angle_temp-M_PI;
//                //MOVE//
//                move(msg.max_displacement, finalAngle);
//                bocas_msg.text = "Foge que vem ai o predador " + hunters_team->players[angleMinHunter];
//                //                } else {
//                //                    ROS_INFO_STREAM("My team proximo: " << hunters_team->players[angleMinHunter] << " angle: " << getAngleToPLayer(hunters_team->players[angleMin]));
//                //                    double angle_temp = getAngleToPLayer(my_team->players[angleMinMy]);
//                //                    finalAngle = angle_temp+M_PI;
//                //                    if (angle_temp > 0)
//                //                        finalAngle = angle_temp-M_PI;
//                //                    //MOVE//
//                //                    move(msg.max_displacement, finalAngle);
//                //                    bocas_msg.text = "Foge que vem ai o predador " + my_team->players[angleMinMy];
//                //                }
//            } else {
//                ROS_INFO_STREAM("Preyer mais proximo: " << preys_team->players[angleMin] << " angle: " << getAngleToPLayer(preys_team->players[angleMin]));
//                //MOVE//
//                move(msg.max_displacement, getAngleToPLayer(preys_team->players[angleMin]));
//                bocas_msg.text = preys_team->players[angleMin] + ", toma toma toma foguetinhos!!!";
//            }
//        }

//        publisher.publish(bocas_msg);

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
