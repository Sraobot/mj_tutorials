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
// double K[4] =  {-39.63348624,   2.92588523,  39.4855769 ,   0.};
double K[4] = {-265.4197, -97.9928, -66.4967, -28.8720}; // Gain calculated from lqr command.
int lqr = 0;

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

void f(const mjModel* m, mjData* d, double input[5], double outputs[4])
{
    // state   - q1, q1dot, q2, q2dot
    // inputs  - q1, q1dot, q2, q2dot, u
    // outputs - q1dot, q1ddot q2dot, q2ddot

    d->qpos[0] = input[0];
    d->qvel[0] = input[1];
    d->qpos[1] = input[2];
    d->qvel[1] = input[3];
    d->ctrl[0] = input[4];

    printf("flag: %d\n", lqr);
    
    mj_forward(m,d);
    
    double q1dot, q2dot;
    q1dot = d->qvel[0];
    q2dot = d->qvel[1];

    // Equations of Motion
    // M*qacc + qfrc_bias = ctrl
    // qacc = inv(M)*(ctrl - qfrc_bias)
    // qacc = q1ddot, q2ddot

    const int nv = 2;
    double M[nv*nv] = {0};
    mj_fullM(m,M,d->qM); // This function stores the result in 'M'.
    
    double detM = M[0]*M[3] - M[1]*M[2];
    double Minv[] = {M[3], -M[1], -M[2], M[0]}; 
    
    for(int i=0; i<4; ++i) { 
        Minv[i] = Minv[i]/detM;
        // printf("%lf  ", Minv[i]);
    }
    // printf(" \n ");
    
    double qacc[nv] {0};
    
    // f = (ctrl - qfrc_bias)
    double f[nv] = {0};
    f[0] = 0 - d->qfrc_bias[0]; // For the first actuator that does not exist(shown by giving u = 0)
    f[1] = d->ctrl[0] - d->qfrc_bias[1];

    mju_mulMatVec(qacc, Minv, f, 2,2);
    
    double q1ddot = qacc[0], q2ddot = qacc[1];
    outputs[0] = q1dot;
    outputs[1] = q1ddot;
    outputs[2] = q2dot;
    outputs[3] = q2ddot;
}

void init_controller(const mjModel *m, mjData *d)
{
    int i = 0;
    double input[5] = {0};
    double output[4] = {0};
    
    f(m,d,input, output);

    // f0 = f(all inputs with zero values)
    // Storing output value in f0 as output will get changed later
    double f0[4] = {0};
    for(i=0; i<4; ++i){
        f0[i] = output[i];
    }

    double pert = 0.0001;
    
    // Calculating A matrix
    double A[4][4] = {0};
    int j=0;
    for(i=0; i<4; ++i)
    {
        input[i] = pert;  // technically it should be input[i] = input[i] + pert. Since input[i]=0, no need;
        f(m,d,input,output);
        for(j=0; j<4; ++j){
            A[j][i] = (output[j]-f0[j])/pert;
        }
        input[i] = 0;
        
    }
    
    for(i=0;i<4;++i){
        for(j=0;j<4;++j){
            printf("%lf  ", A[i][j]);
        }
        printf("\n");
    }
    
    // Calculating B matrix
    double B[4] = {0};
    input[4] = pert;
    f(m,d,input, output);
    for(i=0; i<4; ++i) 
        B[i] = (output[i] - f0[i])/pert;    

    printf("%lf  %lf  %lf  %lf\n",B[0], B[1], B[2], B[3]);
    
}

void init_file(std::ofstream &file) {
    file << "t, x, z " << std::endl;
}

// Controller function. It takes two arguments mjModel and mjData.
// We can put const on model as it does not change. This function gets called as
// t equal to mjcb_control just before main().
void mycontroller(const mjModel *m, mjData *d)
{
    if(lqr) {
        // printf("Inside the controller\n");
        // printf("flag: %d, %lf   %lf   %lf   %lf\n",lqr, d->qpos[0], d->qpos[1], d->qvel[0], d->qvel[1]);
        double noise;
        mju_standardNormal(&noise); // Generates standard normal noise.

        // We can add the noise in control or we can apply external force by using d->xfrc_applied.
        d->ctrl[0] = -K[0]*d->qpos[0] - K[1]*d->qvel[0] - K[2]*d->qpos[1] - K[3]*d->qvel[1] + 2*noise; 
        d->xfrc_applied[6*2+ 0] = 3*noise; // 2 denotes the body. Read comments.txt for more details.
        d->qfrc_applied[0] = noise;
    }
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
    
    // d->qpos[1] = qinit[1];
   
     // Defining the controller function
    mjcb_control = mycontroller;
    
    // Initialising COntroller and the file
    init_controller(m,d);
    lqr = 1;
      
    //init_file(outfile);
    d->qpos[0] = 0.02;
    while (!glfwWindowShouldClose(window))
    {
        mjtNum simstart = d->time;

        while (d->time - simstart < 1.0 / 60.0)
        {
            // printf("flag: %d, %lf   %lf   %lf   %lf\n",lqr, d->qpos[0], d->qpos[1], d->qvel[0], d->qvel[1]);
            mj_step(m, d);
        }

        //writing to the file
        // outfile << d->time << ", " << d->sensordata[0] <<  ", " << d->sensordata[2] << std::endl; 
        
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

        // if(d->time > 10) {
             // printf("\nClosed window");
             // glfwSetWindowShouldClose(window,true);
         // }
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

