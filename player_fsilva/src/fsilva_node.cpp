//ATD includes
#include <iostream>
#include <vector>

//ROS INCLUDES
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//Local includes
#include <rwsua2017_libs/player.h>
#include <rwsua2017_msgs/MakeAPlay.h>


using namespace std;
using namespace boost;
using namespace tf;
using namespace ros;

double randNumber(){
                struct timeval t1;
                gettimeofday(&t1,NULL);
                srand(t1.tv_usec);
                double x =((((double)rand()/(double)RAND_MAX)*2 -1)*5);

                return x;
}

namespace rwsua2017
{

  class MyPlayer: public Player
  {
    public:

      //PROPPERTIES
      Subscriber sub;
      TransformBroadcaster br;
      Transform t1;
      TransformListener listener;
      StampedTransform transf;

      MyPlayer(string argin_name, string argin_team_name): Player(argin_name, argin_team_name)
    {
      //Subscribe tyo the make_a_play_message

          t1.setOrigin(tf::Vector3(randNumber(),randNumber(),0));
          tf::Quaternion q;
          q.setRPY(0,0,randNumber());
          t1.setRotation(q);
          br.sendTransform(tf::StampedTransform(t1, ros::Time::now(),"map",name));

      sub = n.subscribe("/make_a_play/dog", 1000, &MyPlayer::makeAPlayCallback, this);

      cout << "Initialized MyPlayer" << endl;
    };


      void makeAPlayCallback(const rwsua2017_msgs::MakeAPlay::ConstPtr& msg)
      {

        cout << "received a makeAPlay msg" << endl;
        cout << "max_dispalcemente: " << msg->max_displacement << endl;

        float turn_angle = M_PI/10;
        float displacement = msg->max_displacement;

        tf::Transform tmov;
        tf::Quaternion q;
        q.setRPY(0,0,turn_angle);

        tmov.setRotation(q);
        tmov.setOrigin(tf::Vector3(displacement,0,0));

        tf::Transform t = t1 * tmov;

        br.sendTransform(tf::StampedTransform(t1, ros::Time::now(),"map",name));
        t1 = t;

      }

      double getAngleTo(string s)
      {
          try{
                listener.lookupTransform("/fsilva", s, ros::Time(0), transf);
              }
              catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                //continue;
              }

          float angle = atan2(transf.getOrigin().y(),
                        transf.getOrigin().x());
      }

      vector<string> teammates;

  };
}


int main(int argc, char **argv)
{
 // cout << "Hello world" << endl;

  ros::init(argc, argv, "player_fsilva");


  rwsua2017::MyPlayer myplayer("fsilva", "green");

  cout << "name = " << myplayer.name << endl;
  cout << "team name = " << myplayer.get_team_name() << endl;

  myplayer.teammates.push_back("fsilva");
  myplayer.teammates.push_back("vsilva");

  //ciclo for tipico em c
  //int i;
  //for (i=0; i < 5; i = i + 1)

  cout << "teammates:" << endl;
  for (size_t i = 0; i < myplayer.teammates.size(); ++i)
  {
    cout << myplayer.teammates[i] << endl;
  }

  ros::spin();

  return 1;
}
