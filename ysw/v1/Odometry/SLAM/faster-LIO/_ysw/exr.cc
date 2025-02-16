#include <iostream>

#define TEST 1

int main(){
    // laser_mapping::PublishFrameWorld
    int run_in_offline_ = 0;
    int scan_pub_en_ = 1;
    int pcd_save_en_ = 1;
    if (!(run_in_offline_ == false && scan_pub_en_) && !pcd_save_en_) {
        std::cout<<"ah\n";
    }

    return 0;
}