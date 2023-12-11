// =============================================================================
// Adapted from code by Peter G. Adamczyk 
// 2018-10
// Updated 2021-02-27
// Adapted by: Harry K. Dec, 2023
// =============================================================================

#include "path_gen.h"

struct PATHSPEC * path_queue[];
int pathlength; // current max length is 25
struct POSE * est_pose;


void pathLoop(){
// set up a rate basis to keep it on schedule.
    int index;
    int specs_index = (index % QLENGTH);
    int updateDelay = 100;  // 100 ms 10Hz
    while(true){
        path_segment_spec = path_queue[specs_index];
        path_follow(est_pose);
        if(path_is_complete){
            return;
        } else{
            index++;
        }
        cyhal_system_delay_ms(updateDelay);
    }     
}

void refreshWaypoint()
{
    
}
  