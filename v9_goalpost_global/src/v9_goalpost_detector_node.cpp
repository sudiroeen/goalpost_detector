#include "v9_goalpost_global/v9_goalpost_global.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    ros::init(argc, argv, "v9_goalpost_global_node");

    GoalPostDetector goaliDetector_;

    ros::Rate loop_rate(30);

    while(ros::ok()){
        ros::spinOnce();

        double dT = getTickCount();
        goaliDetector_.process();
        cout << "dT: " << (getTickCount() - dT)/getTickFrequency() << endl;
        
        loop_rate.sleep();
    }
    return 0;
}