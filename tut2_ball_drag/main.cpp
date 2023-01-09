#include <cstdio>
#include <cstring>
#include <cmath>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>


char filename[] = "ball.xml";

// Mujoco data structures
mjModel* m = NULL;        // MujoCo model
mjData* d = NULL;	  // Mujcoc Data
mjvCamera cam;		  // abstract camera
mjvOption opt;		  // visualisation options
mjvScene scn;		  // abstract scene
mjrContext con;		  // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dervatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback for OpenGL
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key==GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m,d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right = (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    if(!button_left && !button_middle && !button_right)
        return;
    
    // compute mouse displacement
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS || 
                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
    
    // determine action based on mouse button
    mjtMouse action;
    if(button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;
    
    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);   
}

//scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

// main function
int main(int argc, const char** argv)
{
    char error[1000] = "Could not load binary model";

    // That addition subtraction of numbers is just pointer shifting from starting to end by adding the length and then moving back 
    // the pointer by 4 places to get the extension of the file and then match it.
    //if(std::strlen(argv[1]) > 4 && 
    //   !std::strcmp(argv[1] + std::strlen(argv[1])-4,".mjb")) { 
    //         m = mj_loadModel(argv[1], 0);
    // } else {
       
    // }

    // check command line arguments

    if(argc < 2) {
         m = mj_loadXML(filename, 0 , error, 1000);
    } else {
        mju_error("Wrong usage");
    }
    
    if(!m) {
        mju_error_s("Load model error: %s", error);
    }

    // make data
    d = mj_makeData(m);

    // init GLFW
    if(!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current and request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);

    // initialise mujoco visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // it is just setting up the camera. Nothing else. If we even comment it out then also code works
    // with default configuration.
    double arr_view[] = {90, -45, 3.44, 0.0, 0.0, 0.0};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
    
    // modifying the gravity
    //m->opt.gravity[2] = -1.0;

    // modifying the initial position
    // qpos is nq*1. See comments.txt
    d->qpos[2] = 0.1;

    // modifying the velocity is 6x1
    d->qvel[2] = 5;
    d->qvel[0] = 1;

    // It will show the world frame axis.
    opt.frame = mjFRAME_WORLD;

    while(!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        
        while(d->time - simstart < 1.0/60.0) {
            mj_step(m,d);
            
            //Adding drag force drag_force 
            double vx, vy, vz;
            vx = d->qvel[0]; vy = d->qvel[0]; vz = d->qvel[0];
            double v = sqrt(vx*vx + vy*vy + vz*vz);
            double fx, fy, fz;
            double c = 5.0;
            fx = -c*v*vx;
            fy = -c*v*vy;
            fz = -c*v*vz;
            // Applying an external force
            d->qfrc_applied[0] = fx;
            d->qfrc_applied[1] = fy;
            d->qfrc_applied[2] = fz;
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // Making camera follow the ball
        cam.lookat[0] = d->qpos[0];
        cam.lookat[1] = d->qpos[1];

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f}",cam.azimuth, cam.elevation, cam.distance, 
        //                                    cam.lookat[0], cam.lookat[1], cam.lookat[2]);

        // swap OpenGL buffers
        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    mj_deleteData(d);
    mj_deleteModel(m);

    glfwTerminate();
    return 0;
}
