#include "data_handler.h"
#include "ukf.h"

int main(int argc, char ** argv){

    if(argc < 2){
        std::cerr << "Please pass the file no (1 or 2) as command line argument\n";
        exit(1);
    }

    //first argument is the file_no
    const int file_no{atoi(argv[1])};

    if(file_no > 2 && file_no <= 0){
        std::cerr << "The file number should be either of 1 or 2\n";
        exit(1);
    }

    // create a ukf object and run it
    Ukf quaternion_filter(file_no);
    quaternion_filter.run();

    return 0;
}