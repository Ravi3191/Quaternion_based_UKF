#include "data_parser.h"
#include "ukf.h"

int main(){

    ukf quaternion_filter(2,true);
    quaternion_filter.run();

    return 0;
}