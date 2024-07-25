
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include "yamlLoader.hpp" // yamlLoader loader include file.

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "../model/iiwa14/world.urdf";
const string robot_file = "../model/iiwa14/iiwa14.urdf";
const string robot_name = "iiwa";
const string camera_name = "camera_fixed";

int loopFreq = 100;

// redis keys:
// - write:
const string JOINT_ANGLES_KEY = "sai2::dual_proxy::simviz::sensors::q";
const string JOINT_VELOCITIES_KEY = "sai2::dual_proxy::simviz::sensors::dq";

const std::string EE_FORCE_KEY = "sai2::dual_proxy::simviz::sensors::force";
const std::string EE_MOMENT_KEY = "sai2::dual_proxy::simviz::sensors::moment";

const string ROBOT_SENSED_FORCE_KEY = "sai2::dual_proxy::simviz::sensors::sensed_force";
const string EE_POSE = "sai2::dual_proxy::simviz::sensors::ee_pose";
const string EE_ORIENTATION = "sai2::dual_proxy::simviz::sensors::ee_orientation";

// - read
// const std::string TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string ROBOT_COMMAND_TORQUES_KEY = "sai2::dual_proxy::simviz::actuators::tau_cmd";
const string CONTROLLER_RUNNING_KEY = "sai2::dual_proxy::simviz::controller_running_key";

RedisClient redis_client;

// imgui function prototypes
void imgui_init_setup(GLFWwindow* window);
void imgui_init_frame();
void imgui_render();
void imgui_cleanup();

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, Sai2Graphics::Sai2Graphics* graphics);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);
	// load config file.
	yamlLoader robotconfig("../src/config.yaml");
	
	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	// graphics->showLinkFrame(true, robot_name, "link0", 0.15);
	// graphics->showLinkFrame(true, robot_name, "link7", 0.15);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->updateKinematics();

	// set initial position.
	// VectorXd q_init = VectorXd(0.01, 1.02, -0.7, 1.57, -0.35, 1.3, 3.2);
	VectorXd q_init(7);
	// q_init << 1.01, -1.02, 1.7, 1.57, -0.35, 1.3, 1.03;
	q_init = robotconfig.get_qinit();
	// robot->_q = q_init;
	// robot->updateKinematics();
	// cout << "q: " <<robot->_q << "\n";

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.4);
	sim->setCoeffFrictionDynamic(0.02);

	// sim->setCollisionRestitution(0.5);
	// sim->setCoeffFrictionStatic(0.4);
	// sim->setCoeffFrictionDynamic(0.4);

	sim->setJointPositions(robot_name,q_init); // set q_init as the default pose.
	// sim->setJointVelocities(robot_name,VectorXd::Zero(7));

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateKinematics();

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "Kuka simu", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// Imgui setup
	// imgui_init_setup(window);
	// cache variables
	double last_cursorx, last_cursory;

	redis_client.set(CONTROLLER_RUNNING_KEY, "0");
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim, graphics);
	
	// while window is open:
	while (fSimulationRunning)
	{

		// cout << "q: " <<robot->_q << "\n";
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		
		// imgui frame initialized.
		// imgui_init_frame();
		// imgui_render();
		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}

		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// cleanup imgui
	// imgui_cleanup();

	// destroy context
	glfwSetWindowShouldClose(window,GL_TRUE);
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, Sai2Graphics::Sai2Graphics* graphics) 
{

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd gravity = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);

	// create force sensor
	ForceSensorSim* force_sensor = new ForceSensorSim(robot_name, "link7", Eigen::Affine3d::Identity(), robot);
	ForceSensorDisplay* force_display = new ForceSensorDisplay(force_sensor, graphics);
	Eigen::Vector3d force, moment;
	// force_sensor->enableFilter(0.45);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(loopFreq); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		if (redis_client.get(CONTROLLER_RUNNING_KEY) == "1"){
			gravity.setZero();
		}
		else {
			// cout << "command torques + gravity : \n" << command_torques + gravity << "\n";
			robot->gravityVector(gravity);
			// robot->coriolisPlusGravity(gravity);
		}
		// read arm torques from redis
		command_torques = redis_client.getEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY);

		cout << "command torques: \n" << command_torques << "\n";

		sim->setJointTorques(robot_name,command_torques + gravity);
		sim->integrate(0.01);


		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();
		// robot->updateModel();

		// get ee_pose.
		// Vector3d robot_pos;
		// Matrix3d robot_ori;
		// robot->position(robot_pos,"link7",Vector3d(0, 0, 0.0815));
		// robot->rotation(robot_ori,"link7");
		// update force sensor and display
		force_sensor->update(sim);
		force_sensor->getForce(force);
		force_sensor->getMoment(moment);
		force_display->update();

		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.setEigenMatrixJSON(EE_FORCE_KEY, force);
		redis_client.setEigenMatrixJSON(EE_MOMENT_KEY, moment);

		// redis_client.setEigenMatrixJSON(EE_POSE,robot_pos);
		// redis_client.setEigenMatrixJSON(EE_ORIENTATION,robot_ori);

		//update last time
		// last_time = curr_time;

		simulation_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

void imgui_init_setup(GLFWwindow* window) {
	  //   ---------------------------------------
	//    setup imgui context.
	//   ---------------------------------------
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	ImFontConfig config;
	config.OversampleH = 2;
	config.OversampleV = 1;
	config.GlyphExtraSpacing.x = 1.0f;

	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.Fonts->AddFontFromFileTTF("./fonts/SpaceMono-Regular.ttf",30.0f,nullptr,nullptr);
	// io.Fonts->AddFontFromFileTTF("ProggyClean.ttf",12.0f,&config,nullptr);
	io.Fonts->Build();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // enable keyboard controls.
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
	// setup imgui style.
	ImGui::StyleColorsDark();
	const char* glsl_version = "#version 130";

	// When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

	// setup platform renderer backend.
	ImGui_ImplGlfw_InitForOpenGL(window,true);
	
    ImGui_ImplOpenGL3_Init();

	// imgui states
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    // -------------- end of imgui setup -----------------
}

void imgui_init_frame() {
	// imgui states
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    // -------------- end of imgui setup -----------------
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	if (show_demo_window)
		ImGui::ShowDemoWindow(&show_demo_window);
	{
	ImGui::Begin("Hello world");
	// ImGui::Text(angles.c_str());
	// ImGui::Checkbox("Demo Window", &show_demo_window);
	ImGui::End();
	}

		// 3. Show another simple window.
	if (show_another_window)
	{
		ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::Text("Hello from another window!");
		if (ImGui::Button("Close Me"))
			show_another_window = false;
		ImGui::End();
	}

		// End imgui frame.
}

void imgui_render() {
	// Imgui render.
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	 if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }
}

void imgui_cleanup() {
	// Imgui Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}