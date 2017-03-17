

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <rwsua2017_libs/player.h>
#include <rwsua2017_msgs/MakeAPlay.h>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <boost/make_shared.hpp>


#define MAX_ANGLE M_PI/30

double randNumber(){
                struct timeval t1;
                gettimeofday(&t1,NULL);
                srand(t1.tv_usec);
                double x =((((double)rand()/(double)RAND_MAX)*2 -1)*5);

                return x;
}


using namespace std;

namespace rwsua2017{
        class MyPlayer: public Player{

                public:
                //ros::NodeHandle n;
                ros::Subscriber sub;
                tf::TransformBroadcaster br;

                tf::Transform t1;
                tf::TransformListener listener;

                MyPlayer(string name, string team): Player(name, team){


                        t1.setOrigin(tf::Vector3(randNumber(),randNumber(),0));
                        tf::Quaternion q;
                        q.setRPY(0,0,randNumber());
                        t1.setRotation(q);
                        br.sendTransform(tf::StampedTransform(t1, ros::Time::now(),"map",name));


                        cout << endl;

                        sub = n.subscribe("/make_a_play/dog",1000, &MyPlayer::makeAPlay,this);

                        cout << "Inicialized MyPlayer" << endl;
                }

                vector<string> teamMates;

                void printTeamMates(){
                    for(size_t i = 0; i<teamMates.size(); i++){
                                                cout << teamMates[i] << endl;
                    }
                }

                void makeAPlay(const rwsua2017_msgs::MakeAPlay::ConstPtr& msg)
                {
                  cout << "received a makeAPlay msg" << endl;
                        cout << "max_dispalcemente: " << msg->max_displacement << endl;

                        float turn_angle = M_PI/10;
                        float displacement = msg->max_displacement;

                        tf::Transform tmov;
                        tf::Quaternion q;
                        double angle = getAngleFromTo(name,"moliveira");
                        if(angle > MAX_ANGLE){ angle = MAX_ANGLE;}
                        if(angle < -MAX_ANGLE){ angle = -MAX_ANGLE;}
                        q.setRPY(0,0,angle);

                        tmov.setRotation(q);
                        tmov.setOrigin(tf::Vector3(displacement,0,0));

                        tf::Transform t = t1 * tmov;

                        br.sendTransform(tf::StampedTransform(t1, ros::Time::now(),"map",name));
                        t1 = t;
                }

double getAngleFromTo(string myPlayer, string player){

        tf::StampedTransform transform;

         try{
     listener.lookupTransform(myPlayer, player,
                                                                                                                        ros::Time(0),transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

                double anglle = atan2(transform.getOrigin().y(),
                          transform.getOrigin().x());

                return anglle;
}

/*
                bool isMyTeam(vector<string> team, string teamName){
                                for(int i = 0; i<team.size();i++){
                                                if(team[i] == name){
                                                        cout << "My Team is " << teamName << endl;
                                                        return true;
                                                }
                                }
                                return false;
                }
*/
        };
}


using namespace rwsua2017;

int main(int argc, char **argv)
{

    ros::init(argc,argv,"player_fsilva");
//Creating an instance of class Player
    MyPlayer player("fsilva","blue");
    //player.setTeamName("red");

    cout << "player.name is " << player.name << endl;
    cout << "team is " << player.get_team_name() << endl;

    player.teamMates.push_back("fsilva");
    player.teamMates.push_back("vsilva");

    player.printTeamMates();

    vector< boost::shared_ptr<Player> > teamMates;


    ros::spin();
    return 1;
}
