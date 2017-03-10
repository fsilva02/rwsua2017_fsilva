#include <iostream>
#include <vector>

using namespace std;

namespace  player_fsilva {

    class Player
    {
       public:

        Player(string name, std::string argin_team_name = "green")
        {
            //std::cout <<"player name " << argin_name << std::endl;
            this->name = name;
            set_team_name(argin_team_name);
        }

        string name;

        //acessor (set)
        void set_team_name( string argin_team_name)
        {
            if(argin_team_name == "red" || argin_team_name == "blue" || argin_team_name == "green")
            {
                this->team_name = argin_team_name;
            }
            else
            {
                cout << "Error incorrect team name" << endl;
            }
        }

        void set_team_name(void)
        {
            set_team_name("red");
        }
        //acessor (get)
        string get_team_name(void)
        {
            return this->team_name;
        }
    private:
        string team_name;

    };

    class MyPlayer: public Player
    {
      public:

        MyPlayer(string name, string argin_team_name): Player(name, argin_team_name)
        {
            cout << "Initialized MyPlayer " << endl;
        }

        vector<string> teammates;

    };
}


int main()
{

    //Creating an instance of class Player
    player_fsilva::MyPlayer myplayer("Filipe" , "red");

    cout << "Created an instance of class player with public name " << myplayer.name << endl;
    cout << "name = " << myplayer.name << endl;
    cout << "team name = " << myplayer.get_team_name() << endl;

    myplayer.teammates.push_back("fsilva");
    myplayer.teammates.push_back("vsilva");

    //unsigned long int== size_t
    for (size_t i = 0; i < myplayer.teammates.size(); ++i)
    {
        cout << myplayer.teammates[i] << endl;
    }
}
