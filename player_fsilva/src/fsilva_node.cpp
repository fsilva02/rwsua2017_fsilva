#include <iostream>
#include <vector>

#include <rwsua2017_libs/player.h>

//test include rws message
#include <rwsua2017_msgs/MakeAPlay.h>

// ROS include
#include<ros/ros.h>
//indent your code!

using namespace std;
using namespace boost;

namespace rwsua2017
{

  class MyPlayer: public Player
  {
    public:
     ros::Subscriber sub;

    MyPlayer(string argin_name, string argin_team_name): Player(argin_name, argin_team_name)
    {

      sub = n.subscribe("/make_a_play", 1000, &MyPlayer::makeAPlayCallback, this);
      cout << "Initialized MyPlayer\n" << endl;
    };

    vector<string> teammates;



  void makeAPlayCallback(const rwsua2017_msgs::MakeAPlay::ConstPtr& msg)
  {
    cout << "Message received";
  }
   };
}


int main(int argc, char **argv)
{
 // cout << "Hello world" << endl;
    ros::init(argc, argv, "player_fsilva");

    ;

  rwsua2017::MyPlayer myplayer("player_fsilva", "green");

  cout << "name = " << myplayer.name << endl;
  cout << "team name = " << myplayer.get_team_name() << endl;

  myplayer.teammates.push_back("jferreira");
  myplayer.teammates.push_back("jferreira");

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
