#include <cstdio>
#include <cstring>
#include <cmath>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

char filename[] = "double_pendulum.xml";

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

// Controller helper functions -- 
// 1. set_position_servo   2. set_velocity_servo   3. set_torque_control
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
    mj_energyPos(m,d); // Computes Potential energy
    mj_energyVel(m,d); // Computes kinetic energy
    // These energy can be accessed from the mjData d structure.
    // printf("potential energy: %lf, kinetic energy: %lf total energy: %lf\n", 
    //        d->energy[0], d->energy[1], d->energy[0]+d->energy[1]);
    // Actually, this print will give energy as 0.000 until we enable the energy flag
    // in the xml.

    // Getting the mass matrix(M) which is Mqdotdot + Cqdot + G = tau
    const int nv = 2;
    double dense_M[nv*nv] = {0};  
    mj_fullM(m,dense_M,d->qM); // It stores the mass matrix into dense_M as a array.

    // Getting the accleration
    double qddot[nv] = {0};
    qddot[0] = d->qacc[0];
    qddot[1] = d->qacc[1];

    // Getting f=C(q,qdot)qdot + G(q);
    double f[nv]={0};
    f[0] = d->qfrc_bias[0];
    f[1] = d->qfrc_bias[1];

    // Adding M*qddot + C(q,qdot)*qdot + G(q)
    double lhs[nv] = {0};
    mju_mulMatVec(lhs, dense_M, qddot, 2,2);
    lhs[0] = lhs[0] + f[0];
    lhs[1] = lhs[1] + f[1];

    d->qfrc_applied[0] = f[0];
    d->qfrc_applied[1] = f[1];

    // If our model has no actuator then our model if Mqddot +Cqdot + G = tau
    // Here C+G is combined in one quantity. tau is qfrc_applied. If we make qfrc_applied
    // same as C+G then Mqddot = 0 =>No qddot=0 => q is unchanged.

    double rhs[nv]={0};
    rhs[0] = d->qfrc_applied[0];
    rhs[1] = d->qfrc_applied[1];

    //printf("%f  %f\n", lhs[0], rhs[0]);
    //printf("%f  %f\n", lhs[1], rhs[1]);
    //printf("%f  %f \n", lhs[0], lhs[1]);

    // Now giving a control input. Earlier manipulation was done to show something.
    // Not done for control input.
    double Kp1=100, Kp2=100;
    double Kv1=10, Kv2=10;

    double qref1=-0.5, qref2=-1.57;

    // PD control
    // d->qfrc_applied[0] = -Kp1*(d->qpos[0]-qref1) - Kv1*d->qvel[0];
    // d->qfrc_applied[1] = -Kp1*(d->qpos[1]-qref2) - Kv1*d->qvel[1];

    // coriolis + gravity + PD control
    // d->qfrc_applied[0] = f[0] - Kp1*(d->qpos[0]-qref1) - Kv1*d->qvel[0];
    // d->qfrc_applied[1] = f[1] -Kp1*(d->qpos[1]-qref2) - Kv1*d->qvel[1];
    
    // Feedback linearisation
    // tau = M*(PD) + f . Recall in Simulink, we pre-multiplied with inv(M) and integrated qddot,
    // Here, we cannot do that so we multiply with M to get it going.
    double tau[2] = {0};
    tau[0] = -Kp1*(d->qpos[0]-qref1) - Kv1*d->qvel[0];
    tau[1] = -Kp1*(d->qpos[1]-qref2) - Kv1*d->qvel[1];
    
    // M*PD
    mju_mulMatVec(tau, dense_M, tau, 2,2);

    // Adding f to M*PD
    d->qfrc_applied[0] = tau[0] + f[0];
    d->qfrc_applied[1] = tau[1] + f[1];
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
    //d->qpos[0] = 1;

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
