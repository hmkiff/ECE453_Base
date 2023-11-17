#ifdef SIM

#include <limits.h>
#include <math.h>
#include "tigr.h"
#include "botstate.h"
#include "environment.h"
#include "swarm.h"
#include "simbot.h"

#define M_PI 3.14159265358979323846

enum Initpos {
    LINE,
    CIRCLE
};

void drawBot(Tigr *screen, botstate bot[NUM_BOTS], int bot_ind, TPixel color) {
    // Chassis
    int botRad = 20;
    tigrCircle(screen, bot[bot_ind].pos.pos_x, bot[bot_ind].pos.pos_y, botRad, color);
    // Heading indicator
    float x_head = bot[bot_ind].pos.pos_x + botRad * sin(bot[bot_ind].pos.heading_rad);
    float y_head = bot[bot_ind].pos.pos_y + botRad * cos(bot[bot_ind].pos.heading_rad);
    tigrCircle(screen, x_head, y_head, 5, color);
    // Ultrasonic indicator
    float x_us = bot[bot_ind].pos.pos_x + botRad * sin(bot[bot_ind].pos.heading_rad + bot[bot_ind].servo_ang_rad);
    float y_us = bot[bot_ind].pos.pos_y + botRad * cos(bot[bot_ind].pos.heading_rad + bot[bot_ind].servo_ang_rad);
    tigrCircle(screen, x_us, y_us, 3, color);
}

int main(int argc, char *argv[]) {
    botstate bot[NUM_BOTS];

    Environment env;
    env.wall_on = 1;
    env.wall_x = 10;
    env.wall_y = 10;
    env.wall_w = 980;
    env.wall_h = 780;

    SimbotBehavior behavior;
    behavior.sweep_us = 1;
    behavior.sweep_us_step = 0.2;
    behavior.sweep_us_max = M_PI;
    behavior.sweep_us_min = 0;

    for (int i = 0; i < NUM_BOTS; i++) {
        bot[i] = simbot_init(bot, i, behavior);
    }

    Tigr *screen = tigrWindow(1000, 800, "Swarm-sim", 1);
    int time = 0;
    int frameskip = 100000000;
    while (!tigrClosed(screen)) {
        if (time == INT_MAX) {
            time = 0;
        } else {
            time++;
        }

        if (time % frameskip == 0) {
            // Update simulated sensor data
            for (int i = 0; i < NUM_BOTS; i++) {
                bot[i] = simbot(bot, i, env, behavior);
                // Run into walls
                if (bot[i].pos.pos_x < env.wall_x) {
                    bot[i].pos.pos_x = env.wall_x;
                } else if (bot[i].pos.pos_x > env.wall_x + env.wall_w) {
                    bot[i].pos.pos_x = env.wall_x + env.wall_w;
                }
                if (bot[i].pos.pos_y < env.wall_y) {
                    bot[i].pos.pos_y = env.wall_y;
                } else if (bot[i].pos.pos_y > env.wall_y + env.wall_h) {
                    bot[i].pos.pos_y = env.wall_y + env.wall_h;
                }
            }

            tigrClear(screen, tigrRGB(0x00, 0x00, 0x00));

            if (env.wall_on) {
                tigrRect(screen, env.wall_x, env.wall_y, env.wall_w, env.wall_h, tigrRGB(0xFF, 0x00, 0x00));
            }

            for (int i = 0; i < NUM_BOTS; i++) {
                drawBot(screen, bot, i, tigrRGB(0x00, 0x00, 0xFF));
                bot[i].pos = swarm(bot, i);
                drawBot(screen, bot, i, tigrRGB(0x00, 0xFF, 0x00));
            }

            tigrUpdate(screen);
        }
    }
    tigrFree(screen);
    return 0;
}

#endif