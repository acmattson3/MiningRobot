/**
  Aurora Robotics frontend code:
    This sends commands to drive the robot.
    It interfaces with the pilot's keyboard and joystick (via aurora/ui.h).
    It displays telemetry (via aurora/robot.cpp).
  
  Aurora Robotics, UAF, 2014-2023 (public domain)
*/
#define AURORA_IS_FRONTEND 1

#include <iostream>
#include <cmath>
#include <fstream>
#include <map>

#include "aurora/robot_base.h"
#include "aurora/robot_states.cpp"
#include "aurora/display.h" /* for graphics */
#include "aurora/kinematic_links.cpp"
#include "aurora/network.h"
#include "aurora/ui.h"
#include "aurora/data_exchange.h"

#include "ogl/event.cpp"
#include "osl/socket.cpp"

#include "osl/porthread.h" /* for threading */
#include "osl/porthread.cpp"

#include "aurora/lunatic.h"

MAKE_exchange_marker_reports_depth();
MAKE_exchange_marker_reports_webcam();
MAKE_exchange_backend_state();
MAKE_exchange_moveit_goal();
MAKE_exchange_moveit_plan();

std::map<int, vec3> marker_offsets;

vec3 offset_for_marker(int id) {
    auto it = marker_offsets.find(id);
    if (it != marker_offsets.end()) return it->second;
    return vec3(0,0,0);
}

void load_marker_offsets(const char *fname="marker_offsets.txt") {
    std::ifstream f(fname);
    if (!f) { std::cerr << "No marker offset file " << fname << "\n"; return; }
    int id; float x,y,z; 
    while (f >> id >> x >> y >> z) {
        marker_offsets[id]=vec3(x,y,z);
    }
}

// Send the first detected marker location to MoveIt, with offset
void goto_first_marker() {
    const auto &state = exchange_backend_state.read();
    aurora::robot_coord3D frame = state.loc.get3D();
    aurora::robot_link_coords links(state.joint, frame);

    auto process_reports = [&](const aurora::vision_marker_reports &reps, aurora::robot_link_index cam)
    {
        const auto &cam_coord = links.coord3D(cam);
        for (const auto &r : reps) if (r.is_valid()) {
            aurora::robot_coord3D goal = cam_coord.compose(r.coords);
            vec3 off = offset_for_marker(r.markerID);
            goal.origin += off.x*goal.X + off.y*goal.Y + off.z*goal.Z;
            exchange_moveit_goal.write_begin() = goal;
            exchange_moveit_goal.write_end();
            robotPrintln("MoveIt goal for marker %d offset %.2f %.2f %.2f", r.markerID, off.x, off.y, off.z);
            return true;
        }
        return false;
    };

    bool sent = false;
    if (exchange_marker_reports_depth.updated())
        sent = process_reports(exchange_marker_reports_depth.read(), aurora::link_depthcam);
    if (!sent && exchange_marker_reports_webcam.updated())
        sent = process_reports(exchange_marker_reports_webcam.read(), aurora::link_drivecam);

    if (!sent)
        robotPrintln("MoveIt goal: no marker available");
}


/**
 This class represents everything the front end knows about the robot.
*/
class robot_manager_t 
{
public:
	robot_base robot; // overall integrated current state
	
	robot_telemetry telemetry; // last-known telemetry value
	byte last_telemetry_count;
	double last_telemetry_time;
	
	robot_ui ui; // keyboard interface
	robot_command command; // next-sent command
	double last_command_time;
	robot_comms comms; // network link to back end
	
	
	// Do robot work.
	void update(void);
	
	robot_manager_t() {
		last_telemetry_count=0;
		last_telemetry_time=0;
		last_command_time=0;
	}
};
robot_manager_t robot_manager;

void robot_manager_t::update(void) {
	double time=robotTime();
	
// Run UI
        ui.update(oglKeyMap,robot);
        //robotPrintLines(ui.description);

        if (oglKey('o', "goto marker"))
            goto_first_marker();
	
	if (time>=last_command_time+0.050) 
	{
		if (robotState_requested<state_last)
		{ // request to enter state
			command.command=robot_command::command_state;
			command.power=ui.power;
			command.state=robotState_requested;
			robotPrintln("REQUESTING ROBOT STATE %s",state_to_string(robotState_requested));
			
			if (robotState_requested==state_POP) 
			{
			    /* Only send POP once, don't wait for ack */
			    robotState_requested = state_last;
			}
		} 
		else 
		{ // normal powered driving
			command.command=robot_command::command_power;
			command.power=ui.power;
			command.tuneable=ui.tuneable;
			command.state=state_drive;
		}
		comms.broadcast(command);

		if (robot.state==state_drive) 
		{
			robot.power=ui.power;
		}
		
		last_command_time=robotTime();
	}
	
// Check for a telemetry broadcast
	int n;
	while (0!=(n=comms.available(10))) {
		time=robotTime();
		if (n==sizeof(telemetry)) 
		{ // grab telemetry from backend
			comms.receive(telemetry);
			
			robot=telemetry; // copy over all fields
			
			static int last_state=robot.state;
			if (last_state!=robot.state)
				robotPrintln("Robot entering state %s",state_to_string(robot.state));
			last_state=robot.state;
			
			if (robotState_requested<state_last)
			{ // we asked for a new state--stop asking once it's confirmed
				if (telemetry.ack_state==robotState_requested)
				{
					robotState_requested=state_last; // confirmed.
				}
			}
			
			byte next_count=1+last_telemetry_count;
			int distance=abs(telemetry.count - next_count);
			if (distance>2 && next_count!=1) {
				robotPrintln("Telemetry warning> count mismatch. Expected %d, got %d",
					next_count,telemetry.count);
			}
			if (time>0.50+last_telemetry_time) {
				robotPrintln("Telemetry warning> time gap of %.3f seconds",
					time-last_telemetry_time);
			}
			
			last_telemetry_count=telemetry.count;
			last_telemetry_time=time;
			
		} 
		else {
			robotPrintln("ERROR: TELEMETRY VERSION MISMATCH!  Expected %d or %d, got %d",
				sizeof(telemetry),sizeof(command),n);
		}
	}
	/*
	else {
		robotPrintln("NO TELEMETRY");
	}*/
	if (time>0.50+last_telemetry_time) {
		robotPrintln("Missing backend telemetry--is it running?  Network?");
		
	}
	
	//robotPrintln("Location %.0f,%0.0f,%0.0f",robot.loc.x,robot.loc.y,robot.loc.angle);
	
	
	robot_3D_setup(robot.sensor.frame_pitch);
	robot_3D_draw(robot.joint,robot.sensor.connected_tool());
        if (robot.state > state_autonomy)
        {
           robot_3D_draw(robot.joint_plan,robot.sensor.connected_tool(),0.3f);
        }
    robot_3D_cleanup();
	
	robot_2D_display(robot.loc);
	
	robot_display_autonomy(telemetry.autonomy);

    // Limit this loop to 100Hz (10ms/loop)
    aurora::data_exchange_sleep(10);

}

extern "C" void display(void) {
	robot_display_setup(robot_manager.robot);
	
	robot_manager.update();
	
	robot_display_finish(robot_manager.robot);
	
	glutSwapBuffers();
	glutPostRedisplay();
}


int main(int argc,char *argv[])
{
        load_marker_offsets("autonomy/frontend/marker_offsets.txt");
        glutInit(&argc,argv);
	
	// Set screen size
	int w=1280, h=700;
	for (int argi=1;argi<argc;argi++) {
		if (0==strcmp(argv[argi],"-bench")) {  }
		else if (0==strcmp(argv[argi],"-img")) {  }
		else if (2==sscanf(argv[argi],"%dx%d",&w,&h)) {}
		else printf("Unrecognized argument '%s'!\n",argv[argi]);
	}
	glutInitDisplayMode(GLUT_RGBA + GLUT_DOUBLE);
	glutInitWindowSize(w,h);
	glutCreateWindow("Robot Front End");
	
	robotMainSetup();
	
	glutDisplayFunc(display);
	glutMainLoop();
	return 0;
}
