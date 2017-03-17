//math includes
#include <math.h>

//ATD includes
#include <iostream>
#include <vector>

//ROS INCLUDES
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

//Local includes
#include <rwsua2017_libs/player.h>
#include <rwsua2017_msgs/MakeAPlay.h>

double generateRandomCoordinate(){
    struct timeval t1;
    gettimeofday(&t1,NULL);
    srand(t1.tv_usec);
    double x =((((double)rand()/(double)RAND_MAX)*2 -1)*5);

    return x;
}

using namespace std;
using namespace boost;
using namespace tf;
using namespace ros;

namespace rwsua2017
{

class MyPlayer: public Player //herda se a myplayer da classe player
{
public:
    //Properties
    Subscriber sub;
    //variavel de publicacao : broadcaster
    TransformBroadcaster br;
    tf::TransformListener listener;
    ros::Publisher vis_pub;


    MyPlayer(string argin_name, string argin_team_name): Player(argin_name, argin_team_name)
    {

        Transform t1; //cria se uma matriz de transformacao t1

        sub = n.subscribe("/make_a_play/cat", 1000, &MyPlayer::makeAPlayCallback,this);

        vis_pub = n.advertise<visualization_msgs::Marker>( "/bocas", 0 );

        //Poe o nosso jogador na posicao inicial
        // set translacao

        t1.setOrigin(tf::Vector3(generateRandomCoordinate(), generateRandomCoordinate(), 0.0) ); //define se a origem (componente translacao) -> (1,1,0)
        Quaternion q; //serve para representar a rotacao
        //set rotacao
        q.setRPY(0, 0, 0); //seta o quaternion
        t1.setRotation(q);
        br.sendTransform(tf::StampedTransform(t1, ros::Time::now(), "map", name)); //transformacao de onde para onde? mapa para o nome do jogador

        cout << "Initalized MyPlayer" <<endl;
    };

    tf::StampedTransform getPose(float time_to_wait = 0.1)    {
        tf::StampedTransform trans;
        ros::Time now = Time(0);
        try
        {
            listener.waitForTransform("map", name, now, Duration(time_to_wait));
            listener.lookupTransform("map",name,now, trans);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.01).sleep();
        }
        float x = trans.getOrigin().x();
        return trans;
    }

    void makeAPlayCallback(const rwsua2017_msgs::MakeAPlay::ConstPtr& msg)
    {
        cout << "received a make a play msg with max displacement = " << msg->max_displacement <<endl;

        //definicao angulos de rotacao e valores de translacao
        //devia ser calculado pela ai do sys
        // float turn_angle = getAngleToPlayer("vsilva");
        float displacement = 0.5;
        //move my player

        //Distancia e angulos dos que me caçam
        float distanceTomoliveira = getDistanceTo("moliveira");
        float distanceTobrocha = getDistanceTo("brocha");
        float distanceTobvieira = getDistanceTo("bvieira");

        float angleTomoliveira = getAngleToPlayer("moliveira");
        float angleTobrocha = getAngleToPlayer("brocha");
        float angleTobvieira = getAngleToPlayer("bvieira");

        //Distancia e angulo para o mapa
        float distanceToMap = getDistanceTo("map");
        float angleToMap = getAngleToPlayer("map");

        // Distancia e angulo dos que eu caço
        float distanceTovsilva = getDistanceTo("vsilva");
        float distanceTojsousa = getDistanceTo("jsousa");
        float distanceTodcorreira = getDistanceTo("dcorreia");
        float angleTovsilva = getAngleToPlayer("vsilva");
        float angleTojsousa = getAngleToPlayer("jsousa");
        float angleTodcorreia = getAngleToPlayer("dcorreia");

        float angle_inimigo;


        if(distanceTovsilva < distanceTojsousa && distanceTovsilva < distanceTodcorreira)
            angle_inimigo = angleTovsilva;
        if(distanceTojsousa < distanceTovsilva && distanceTojsousa < distanceTodcorreira)
            angle_inimigo = angleTojsousa;
        if(distanceTodcorreira < distanceTojsousa && distanceTodcorreira < distanceTovsilva)
          angle_inimigo = angleTodcorreia;
/*
        if(distanceTobvieira < distanceTobrocha && distanceTobvieira < distanceTomoliveira && distanceTobvieira <= 2)
            move(displacement,-angleTobvieira,msg->max_displacement,M_PI/30);
        else if(distanceTomoliveira < distanceTobrocha && distanceTomoliveira < distanceTobvieira && distanceTomoliveira <= 2)
            move(displacement,-angleTomoliveira,msg->max_displacement,M_PI/30);
        else if(distanceTobrocha < distanceTobvieira && distanceTobrocha < distanceTomoliveira && distanceTobrocha <= 2)
            move(displacement,-angleTobrocha,msg->max_displacement,M_PI/30);
        else
            move(displacement,angle_inimigo,msg->max_displacement,M_PI/30);
        if(distanceToMap >= 5.5)
            move(displacement,angleToMap+M_PI/3,msg->max_displacement,M_PI/30);
*/

        move(displacement,angle_inimigo,msg->max_displacement,M_PI/30);

        // enviar boca
        visualization_msgs::Marker marker;
        marker.header.frame_id = name; //sistema de referencias
        marker.header.stamp = ros::Time();
        marker.ns = name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0.1;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.6;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.4; //when is text, only scale z is used
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.3;
        marker.color.g = 0.3;
        marker.color.b = 0.3;
        marker.frame_locked = 1;
        marker.lifetime = ros::Duration(1);
        marker.text = "potatoes";
        vis_pub.publish( marker ); //publicar o marcador


    }

    float getAngleToPlayer(string player_name,float time_to_wait = 0.1)
    {
        ros::Time now = Time(0);

        tf::StampedTransform trans;
        try
        {
            listener.waitForTransform(name, player_name, now, Duration(time_to_wait));
            listener.lookupTransform(name, player_name,now, trans);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.01).sleep();
        }

        float x = trans.getOrigin().x();
        float y = trans.getOrigin().y();
        cout << "delta x: " << x << endl;
        cout << "delta y: " << y << endl;

        return atan2(y,x);
    }

    float getDistanceTo(string player_name,float time_to_wait = 0.1)
    {
        ros::Time now = Time(0);

        tf::StampedTransform trans;
        try
        {
            listener.waitForTransform(name, player_name, now, Duration(time_to_wait));
            listener.lookupTransform(name, player_name,now, trans);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.01).sleep();
        }

        float x = trans.getOrigin().x();
        float y = trans.getOrigin().y();

        float distance = sqrt(x*x+y*y);
        return distance;

    }

    void move(float displacement,float turn_angle,float max_displacement,float max_turn_angle)
    {
        //saturo o angulo para estar dentro das regras do jogo
        //Assegura que nao se roda mais do que o maximo
        double max_t =  max_turn_angle;
        if (turn_angle > max_t) turn_angle = max_t;
        else if (turn_angle < -max_t) turn_angle = -max_t;


        // Saturacao displacement
        //Assegura que nao se danda mais do que o maximo

        if(displacement > max_displacement)
        {
            displacement = max_displacement;
        }

        //Compute the new reference frame
        tf::Transform t_mov;
        Quaternion q;
        q.setRPY(0, 0, turn_angle);
        t_mov.setRotation(q);
        t_mov.setOrigin( Vector3(displacement , 0.0, 0.0) );
        //t1 memoria local que o codigo tem de onde o jogador estava
        //necessario actualizar t1 consoante o ROS
        //  t1 = getPose();
        tf::Transform t = getPose()  * t_mov;
        //Send the new transform to ROS
        br.sendTransform(StampedTransform(t, ros::Time::now(), "map", name));
    }

    //Lista de strings com o nome dos outros jogadores da minha equipa
    vector<string> teammates;
};
}

int main(int argc, char **argv)
{
    cout << "Hello world" << endl;

    ros::init(argc, argv, "player_fsilva_node");
    string player_name = "fsilva";

    rwsua2017::MyPlayer myplayer(player_name,"green");
    cout << "player: " << myplayer.name << endl;
    cout << "team name = " << myplayer.get_team_name() << endl;

    ros::spin();

    return 1;
}
