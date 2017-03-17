#include <iostream>
#include <vector>
#include <math.h>

//ROS INCLUDES
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

//Local includes
#include <rwsua2017_libs/player.h>
#include <rwsua2017_msgs/MakeAPlay.h>


using namespace std;
using namespace boost;
using namespace tf;
using namespace ros;

namespace rwsua2017
{

  class MyPlayer: public Player
  {
    public:

      //PROPPERTIES
      Subscriber sub;
      tf::TransformListener listener;
      TransformBroadcaster br;
      ros::Publisher vis_pub;

      MyPlayer(string argin_name, string argin_team_name): Player(argin_name, argin_team_name)
    {
      //Subscribe tyo the make_a_play_message
      sub = n.subscribe("/make_a_play/cat", 1000, &MyPlayer::makeAPlayCallback, this);
         vis_pub = n.advertise<visualization_msgs::Marker>( "/bocas", 0 );
      Transform t1;
      t1.setOrigin( tf::Vector3(randNumber(),randNumber(), 0.0) );
      Quaternion q;
      q.setRPY(0, 0, 0);
      t1.setRotation(q);
      br.sendTransform(tf::StampedTransform(t1, ros::Time::now(), "map", name));

      cout << "Initialized MyPlayer" << endl;
    };

      double randNumber(){
        struct timeval t1;
        gettimeofday(&t1,NULL);
        srand(t1.tv_usec);
        double x =((((double)rand()/(double)RAND_MAX)*2 -1)*5);

        return x;
      }

      float getAngleTo(string player_name, float time_to_wait = 0.1)
      {
        tf::StampedTransform trans;
        ros::Time now = Time(0);

        try{
            listener.waitForTransform("map", name, now, Duration(time_to_wait));
            listener.lookupTransform(name, player_name, ros::Time(0), trans);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(0.01).sleep();
        }

        float x = trans.getOrigin().x();
        float y = trans.getOrigin().y();

        return atan2(y,x);

      }

      float getDistanceTo(string player_name, float time_to_wait = 0.1)
      {
        tf::StampedTransform trans;
        ros::Time now = Time(0);

        try{
            listener.waitForTransform("map", name, now, Duration(time_to_wait));
            listener.lookupTransform(name, "player_name", ros::Time(0), trans);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(0.01).sleep();
        }

        float x = trans.getOrigin().x();
        float y = trans.getOrigin().y();
        float result=sqrt(x*x + y*y);
        return result;
      }
      float getDistanceToMap( float time_to_wait = 0.1)
      {
        tf::StampedTransform trans;
        ros::Time now = Time(0);

        try{
            listener.waitForTransform("map", name, now, Duration(time_to_wait));
            listener.lookupTransform(name, "map", ros::Time(0), trans);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(0.01).sleep();
        }

        float x = trans.getOrigin().x();
        float y = trans.getOrigin().y();
        float result=sqrt(x*x + y*y);
        return result;
      }

      tf::StampedTransform getPose(float time_to_wait = 0.1)
      {
        tf::StampedTransform transf;
        ros::Time now = Time(0);

        try{
            listener.waitForTransform("map", name, now, Duration(time_to_wait));
            listener.lookupTransform("map", name, ros::Time(0), transf);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(0.01).sleep();
        }

        return transf;

      }


      void makeAPlayCallback(const rwsua2017_msgs::MakeAPlay::ConstPtr& msg)
      {
        float turn_angle = getAngleTo("moliveira");

        // obter a distancia minima aos adversários
        float distance1 = getDistanceTo("moliveira");
        float distance2 = getDistanceTo("bvieira");
        float distance3 = getDistanceTo("brocha");
        float distance =  std::min(std::min(distance1, distance2), distance3);
        if(distance1 <= distance2 && distance1 <= distance3)
        {
            float turn_angle = getAngleTo("moliveira");
            distance  = distance1;
        }
        if(distance2 <= distance1 && distance2 <= distance3)
        {
            float turn_angle = getAngleTo("bvieira");
            distance  = distance2;
        }
        if(distance3 <= distance2 && distance3 <= distance1)
        {
            float turn_angle = getAngleTo("brocha");
            distance  = distance3;
        }
        float displacement = 0.5;

        move(displacement, turn_angle,distance, msg->max_displacement, M_PI/30);

        //enviar boca
        visualization_msgs::Marker marker;
        marker.header.frame_id = name;
        marker.header.stamp = ros::Time();
        marker.ns = name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0; marker.pose.position.y = 0.4; marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.4;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.3;
        marker.color.g = 0.3;
        marker.color.b = 0.3;
        marker.frame_locked = 1;
        marker.lifetime = ros::Duration(1);
        marker.text = "nao percebes nada";
        vis_pub.publish(marker);

      }

      void move(float displacement, float turn_angle,float distance, float max_displacement,float max_turn_angle)
      {
          if (displacement > max_displacement)
          {
              displacement = max_displacement;
          }

          if (distance < 1.5)
          {
              turn_angle = -turn_angle;
          }
          else
          {
              turn_angle= getAngleTo("vsilva");
          }

          float distance_to_map = getDistanceToMap();
          if (distance_to_map > 5.5)
          {
              turn_angle = -getAngleTo("map");
          }

          double max_t =  (M_PI/30);
          if (turn_angle > max_t) turn_angle = max_t;
          else if (turn_angle < -max_t) turn_angle = -max_t;

          //Compute the new reference frame
          tf::Transform t_mov;
          Quaternion q;
          q.setRPY(0, 0, turn_angle);
          t_mov.setRotation(q);
          t_mov.setOrigin( Vector3(displacement , 0.0, 0.0) );

          tf::Transform t = getPose()  * t_mov;
          //Send the new transform to ROS
          br.sendTransform(StampedTransform(t, ros::Time::now(), "/map", name));
      }

      vector<string> teammates;



  };
}

int main(int argc, char **argv)
{
  cout << "Hello world" << endl;

  ros::init(argc, argv, "fsilva");


  rwsua2017::MyPlayer myplayer("fsilva", "green");

  cout << "name = " << myplayer.name << endl;
  //cout << "team name = " << myplayer.get_team_name() << endl;

  for (size_t i = 0; i < myplayer.teammates.size(); ++i)
  {
    cout << myplayer.teammates[i] << endl;
  }

  ros::spin();

  return 1;
}
