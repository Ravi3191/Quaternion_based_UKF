#include "data_handler.h"
#include "ukf.h"
#include <getopt.h>

int main(int argc, char ** argv){

    if(argc < 3){
        std::cerr << "Argument format: [-f file_number] [-v vicon_availability] \n";
        exit(1);
    }

    int opt;
    bool gt_available(false);
    int file_no{-1};

    while((opt = getopt(argc, argv, "vf:")) != -1){

        switch (opt){
        case 'v':
            gt_available = true;
            break;

        case 'f':
            file_no = atoi(optarg);
            break;

        default:
            break;
        }
    }

    if(file_no <= 0){
        std::cerr << "Did not receive a file number or received an invalid integer \n";
        exit(1);
    }

    // create a ukf object and run it
    Ukf quaternion_filter(file_no,gt_available);
    quaternion_filter.run();

    return 0;
}