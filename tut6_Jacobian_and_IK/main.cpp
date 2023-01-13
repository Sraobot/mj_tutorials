// Uses trajectory control.

#include <cstdio>
#include <cstring>
#include <math.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <fstream>

char filename[] = "double_pendulum.xml";

// simulation and time
double qinit[2] = {0.6, 1.5};
double simend = 20;
double r = 0.25;
double omega = 0.5;

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

// mjtNum kp = 200, kv = 40;

// keyboard callback for OpenGL
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods);

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos);

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset);

void init_controller(const mjModel *m, mjData *d)
{
    mj_forward(m,d);
    // printf("%lf    %lf\n", d->sensordata[0], d->sensordata[2]);
    double r = 0.25;
    // x_pos = x_cen + r => x_cen = x_pos - r
    d->userdata[0] = d->sensordata[0] - r;  // denotes x of circle
    d->userdata[1] = d->sensordata[2];      // denotes y of circle
    // d->userdata[2] = r;

    // printf("userdata_init: %lf %lf\n", d->sensordata[0]-r, d->sensordata[2]);
    // printf("userdata: %lf  %lf\n\n\n",d->userdata[0], d->userdata[1]);
}

void init_file(std::ofstream &file)
{
    file << "t, x, z " << std::endl;
}

// Controller function. It takes two arguments mjModel and mjData.
// We can put const on model as it does not change. This function gets called as
// t equal to mjcb_control just before main().
void mycontroller(const mjModel *m, mjData *d)
{
    double jacp[6] = {0};
    double point[3] = {d->sensordata[0], d->sensordata[1], d->sensordata[2]}; // x,y,z global position of the site
    int body = 2; //0 is ground, 1 is link1 and 2 is link2.
    mj_jac(m, d, jacp, NULL, point, body); // Answer will get stored into jacp;
    // The jacobian we get is 3x2. Each row for x, y and z. Also, as the manipulator moves in x-z plane
    // we will get second row of zeros.
    // printf("%lf  %lf\n", jacp[0], jacp[1]);
    // printf("%lf  %lf\n", jacp[2], jacp[3]);
    // printf("%lf  %lf\n", jacp[4], jacp[5]);
    // printf("\n\n");

    // double qdot[2] = {d->qvel[0], d->qvel[1]};
    // double xdot[2] = {0};
    double J[4] = {jacp[0], jacp[1], jacp[4], jacp[5]};
    // mju_mulMatVec(xdot, J, qdot, 2,2);
    // printf("Velocity using jacobian: %lf  %lf\n", xdot[0], xdot[1]);
    // printf("Velocity using sensordata: %lf  %lf\n", d->sensordata[3], d->sensordata[5]);
    // printf("\n");
    //d->ctrl[0] = qinit[0];
    //d->ctrl[2] = qinit[1];

    double detJ = J[0]*J[3] - J[1]*J[2];
    // printf("detJ: %lf\n", detJ);
    double Jtemp[4] = {J[3], -J[1], -J[2], J[0]};
    double Jinv[4] = {0};
    for(int i=0; i<4; ++i)
        Jinv[i] = Jtemp[i]/detJ;

    // dq = Jinv*dx
    // Change in x
    double x, y;
    x = d->userdata[0] + r*cos(omega*d->time);
    y = d->userdata[1] + r*sin(omega*d->time);

    double dr[] = {x - d->sensordata[0], y- d->sensordata[2]};
    double dq[2] = {};

    mju_mulMatVec(dq,Jinv,dr,2,2);

    d->ctrl[0] = d->qpos[0] + dq[0];
    d->ctrl[2] = d->qpos[1] + dq[1];   
}

// main function
int main(int argc, const char **argv)
{
    char error[1000] = "Could not load binary model";

    // check command line arguments

    if (argc < 2) 
        m = mj_loadXML(filename, 0, error, 1000);
    else 
        mju_error("Wrong usage");
    

    if (!m) 
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // creating the file for data
    std::ofstream outfile;
    outfile.open("data.csv",std::ios_base::app);

    // See ios::trunc and ios::out    
    // init GLFW
    if (!glfwInit()) {
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
    double arr_view[] = {-90, 5, 5, 0.012768, 0.0, 1.254336};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // modifying the initial position, initial state.
    // qpos is nq*1. See comments.txt
    d->qpos[0] = qinit[0];
    d->qpos[1] = qinit[1];

    // It will show the world frame axis.
    //opt.frame = mjFRAME_BODY;
 
    // Defining the controller function
    mjcb_control = mycontroller;

    // Modifying the time step
    //m->opt.timestep = 0.01;

    m->nuserdata = 2;
    // printf("nuserdata: %d\n", m->nuserdata);
    
    // Initialising COntroller and the file
    init_controller(m,d);  
    init_file(outfile);
    
    while (!glfwWindowShouldClose(window))
    {
        mjtNum simstart = d->time;

        while (d->time - simstart < 1.0 / 60.0)
        {
            mj_step(m, d);
        }

        //writing to the file
        outfile << d->time << ", " << d->sensordata[0] <<  ", " << d->sensordata[2] << std::endl; 
        
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

        if(d->time > 10) {
             printf("\nClosed window");
             glfwSetWindowShouldClose(window,true);
         }
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);


    mj_deleteData(d);
    mj_deleteModel(m);

    glfwTerminate();
    return 0;
}

void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    //printf("In keyboard callback");
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        //printf("Inside if condition of backspace calback");
        mj_resetData(m, d);
        mj_forward(m, d);
    }
    else if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE){
        glfwSetWindowShouldClose(window, true);
    }
}


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

