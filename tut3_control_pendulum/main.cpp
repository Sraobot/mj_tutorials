#include <cstdio>
#include <cstring>
#include <cmath>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>


char filename[] = "one_link_pendulum.xml";

// Mujoco data structures
mjModel *m = NULL; // MujoCo model
mjData *d = NULL;  // Mujcoc Data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualisation options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

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
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    //printf("In keyboard callback");
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        //printf("Inside if condition of backspace calback");
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    if (!button_left && !button_middle && !button_right)
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
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// Cnntroller helper functions.
void set_position_servo(const mjModel *m, int actuator_num, double kp)
{
    m->actuator_gainprm[10 * actuator_num + 0] = kp;
    m->actuator_biasprm[10 * actuator_num + 1] = -kp;
}

void set_velocity_servo(const mjModel *m, int actuator_num, double kv)
{
    m->actuator_gainprm[10 * actuator_num + 0] = kv;
    m->actuator_biasprm[10 * actuator_num + 2] = -kv;
}

void set_torque_control(const mjModel* m, int actuator_num, double gain)
{
    m->actuator_gainprm[10*actuator_num + 0] = gain;
}
// Controller function. It takes two arguments mjModel and mjData.
// We can put const on model as it does not change. This function gets called as
// made it equal to mjcb_control just before main().
void mycontroller(const mjModel *m, mjData *d)
{
    // See model.txt file for the indexes.
    // Here, 0 for torque, 1 for position and 2 for velocity.

    // 0=torque
    // Here the qpos is like angle.
    d->ctrl[0] = -10*(d->qpos[0]-3.14) - 2*(d->qvel[0]); // PD control using accurate position and velocity
    return;
    // d->ctrl[0] = -10*(d->sensordata[0] - 0) - 2*(d->sensordata[1] - 0);

    // 1 = position controller
    //d->ctrl[1] = -3; // We need to make kp nonzero in xml file.

    // 2= velocity servo
    //d->ctrl[2] = 1; //

    
    // We did this to find out which of the 10 positions have the kp and Kv value.
    // By doing this, we can change them while the code is running.
    // We have 10*actuator_num. First is for position controller and 2 is the
    // velocity as specified above.
    // We need to modify m->actuator_gainprm and m->actuator_biasprm. 
    // For ---position---, gainprm is at i=0 and biasprm is at i=1 index
    // For ---velocity---, gainprm is at i=0 and biasprm is at i=2 index.
    // printf("position data\n");
    // for (int i = 0; i < 10; ++i)
    // {
    //     printf("%f ", m->actuator_gainprm[10 * 1 + i]);
    //     printf(",   %f \n", m->actuator_biasprm[10 * 1 + i]);
    // }
    /*
    * printf("position data\n");
    * for (int i = 0; i < 10; ++i)
    * {
    *     printf("%f ", m->actuator_gainprm[10 * 1 + i]);
    *    printf(",   %f \n", m->actuator_biasprm[10 * 1 + i]);
    * }
    * 
    * printf("velocity data\n");
    * for (int i = 0; i < 10; ++i)
    * {
    *     printf("%f ", m->actuator_gainprm[10 * 2 + i]);
    *     printf(",   %f \n", m->actuator_biasprm[10 * 2 + i]);
    * }
    * printf("\n");
    */
    
    // If we only set position and velocity servo then the controller takes it to the default position.
    //set_position_servo(m, 1, 40);
    //set_velocity_servo(m, 2, 10);
    
    // d->ctrl[1] = -5.0 - d->qpos[0];
    // d->ctrl[2] = -d->qvel[0];
    //d->ctrl[0] = 0;
    d->ctrl[1] = 0;
    d->ctrl[2] = 0;
    //printf("*******************************");
    printf("%lf \n", *(d->actuator_force));
    // printf("d->ctrl[0] %f", d->ctrl[0]);
    // printf(",  d->ctrl[1] %f", d->ctrl[1]);
    // printf(",  d->ctrl[2] %f", d->ctrl[2]);
    //printf("%lf:",*(d->qfrc_actuator));
    //printf("***********************\n");
    // Setting the positions.

}

// main function
int main(int argc, const char **argv)
{
    char error[1000] = "Could not load binary model";

    // That addition subtraction of numbers is just pointer shifting from starting to end by adding the length and then moving back
    // the pointer by 4 places to get the extension of the file and then match it.
    // if(std::strlen(argv[1]) > 4 &&
    //   !std::strcmp(argv[1] + std::strlen(argv[1])-4,".mjb")) {
    //         m = mj_loadModel(argv[1], 0);
    // } else {

    // }

    // check command line arguments

    if (argc < 2)
    {
        m = mj_loadXML(filename, 0, error, 1000);
    }
    else
    {
        mju_error("Wrong usage");
    }

    if (!m)
    {
        mju_error_s("Load model error: %s", error);
    }

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    // create window, make OpenGL context current and request v-sync
    GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
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
    double arr_view[] = {90, -5, 5, 0.012768, 0.0, 1.254336};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // modifying the gravity
    // m->opt.gravity[0] = 0;
    // m->opt.gravity[1] = 0;
    // m->opt.gravity[2] = -10;

    // modifying the initial position
    // qpos is nq*1. See comments.txt
    d->qpos[0] = 1;

    // modifying the velocity is 6x1
    // d->qvel[2] = 5;
    // d->qvel[0] = 1;

    // It will show the world frame axis.
    //opt.frame = mjFRAME_BODY;

    // Defining the controller function
    mjcb_control = mycontroller;

    // Modifying the time step
    //m->opt.timestep = 0.01;

    while (!glfwWindowShouldClose(window))
    {
        mjtNum simstart = d->time;

        while (d->time - simstart < 1.0 / 60.0)
        {
            mj_step(m, d);
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // Making camera follow the ball
        // cam.lookat[0] = d->qpos[0];
        // cam.lookat[1] = d->qpos[1];

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // printf("{%f, %f, %f, %f, %f, %f}",cam.azimuth, cam.elevation, cam.distance,
        //                                     cam.lookat[0], cam.lookat[1], cam.lookat[2]);

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
