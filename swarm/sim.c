#ifdef SIM

#include <limits.h>
#include <math.h>
#include "tigr.h"
#include "botstate.h"
#include "swarm.h"
#define M_PI 3.14159265358979323846

#define NUM_BOTS 4
#define SIM_BOTS 3

enum Initpos {
    LINE,
    CIRCLE
};

enum Terrain {
    WALL_ONLY
};

int main(int argc, char *argv[]) {
    botstate bot[NUM_BOTS];

    enum Initpos init_pos = LINE;
    if (init_pos == LINE) {
        for (int i = 0; i < NUM_BOTS; i++) {
            bot[i].pos.pos_x = i * 100 + 350;
            bot[i].pos.pos_y = 100;
        }
    }

    enum Terrain terrain = WALL_ONLY;
    int wall_x = 10;
    int wall_y = 10;
    int wall_w = 980;
    int wall_h = 780;

    Tigr *screen = tigrWindow(1000, 800, "Swarm-sim", 1);
    int time = 0;
    int frameskip = 1000000;
    while (!tigrClosed(screen)) {
        if (time == INT_MAX) {
            time = 0;
        } else {
            time++;
        }

        if (time % frameskip == 0) {
            // Update simulated sensor data
            for (int i = NUM_BOTS; i > SIM_BOTS; i--) {
                printf("Swarm: Updating simulated bot %i\r\n", i);
                if (terrain == WALL_ONLY) {
                    float dist_x, dist_y;
                    float ang_x, ang_y;
                    if (bot[i].pos.pos_x < (wall_w / 2)) {
                        dist_x = bot[i].pos.pos_x - wall_x;
                        ang_x = M_PI;
                    } else {
                        dist_x = (wall_w + wall_x) - bot[i].pos.pos_x;
                        ang_x = 0;
                    }
                    if (bot[i].pos.pos_y < (wall_h / 2)) {
                        dist_y = bot[i].pos.pos_y - wall_y;
                        ang_y = M_PI * 0.5;
                    } else {
                        dist_y = (wall_h + wall_y) - bot[i].pos.pos_y;
                        ang_y = M_PI * 1.5;
                    }
                    bot[i].us_dist_cm = (dist_x < dist_y) ? dist_x : dist_y;
                    bot[i].us_servo_ang_rad = (dist_x < dist_y) ? ang_x : ang_y;
                }
            }

            tigrClear(screen, tigrRGB(0x00, 0x00, 0x00));

            if (terrain == WALL_ONLY) {
                tigrRect(screen, wall_x, wall_y, wall_w, wall_h, tigrRGB(0xFF, 0x00, 0x00));
            }

            for (int i = 0; i < NUM_BOTS; i++) {
                tigrCircle(screen, bot[i].pos.pos_x, bot[i].pos.pos_y, 20, tigrRGB(0x00, 0x00, 0xFF));
                bot[i].pos = swarm(bot, i);
                tigrCircle(screen, bot[i].pos.pos_x, bot[i].pos.pos_y, 20, tigrRGB(0x00, 0xFF, 0x00));
            }

            tigrUpdate(screen);
        }
    }
    tigrFree(screen);
    return 0;
}

#endif