/*============================================================================
 Name        : Main.cpp / Project Highway Driving
 Author      : Jean-Yves Bourdoncle
 Version     : v1.0
 Date		 : 05/06/2019
 Description : the implemented functionnalities in this module are :
				1) Migration from the Udacity fonctionalities (from helper.h) :
						- Frenet/Cartesian and Cartesian/Frenet Transformation
						- ClosestWaypoint, calculate the closest waypoint to current x, y position
						- NextWaypoint, Return next waypoint of the closest waypoint
				2) Register the sensor_fusion data in the vehicle class [id,x,y,vx,vy,s,d]
				3) Behavior Planning : ACC, Preparation Lane Change, Lane changer
				4) Spline Creation
				5) Waypoint planner
============================================================================*/

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

///////////////////////////////////////////////////////////////////////////////
///////////////////////UDACITY Fonctions///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
auto found_null = s.find("null");
auto b1 = s.find_first_of("[");
auto b2 = s.find_first_of("}");
if (found_null != string::npos) {
return "";
} else if (b1 != string::npos && b2 != string::npos) {
return s.substr(b1, b2 - b1 + 2);
}
return "";
}

double distance(double x1, double y1, double x2, double y2)
{
return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

double closestLen = 100000; //large number
int closestWaypoint = 0;

for(int i = 0; i < maps_x.size(); i++)
{
double map_x = maps_x[i];
double map_y = maps_y[i];
double dist = distance(x,y,map_x,map_y);
if(dist < closestLen)
{
closestLen = dist;
closestWaypoint = i;
}

}

return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

double map_x = maps_x[closestWaypoint];
double map_y = maps_y[closestWaypoint];

double heading = atan2((map_y-y),(map_x-x));

double angle = fabs(theta-heading);
angle = min(2*pi() - angle, angle);

if(angle > pi()/4)
{
closestWaypoint++;
if (closestWaypoint == maps_x.size())
{
closestWaypoint = 0;
}
}

return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

int prev_wp;
prev_wp = next_wp-1;
if(next_wp == 0)
{
prev_wp = maps_x.size()-1;
}

double n_x = maps_x[next_wp]-maps_x[prev_wp];
double n_y = maps_y[next_wp]-maps_y[prev_wp];
double x_x = x - maps_x[prev_wp];
double x_y = y - maps_y[prev_wp];

// find the projection of x onto n
double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
double proj_x = proj_norm*n_x;
double proj_y = proj_norm*n_y;

double frenet_d = distance(x_x,x_y,proj_x,proj_y);

//see if d value is positive or negative by comparing it to a center point

double center_x = 1000-maps_x[prev_wp];
double center_y = 2000-maps_y[prev_wp];
double centerToPos = distance(center_x,center_y,x_x,x_y);
double centerToRef = distance(center_x,center_y,proj_x,proj_y);

if(centerToPos <= centerToRef)
{
frenet_d *= -1;
}

// calculate s value
double frenet_s = 0;
for(int i = 0; i < prev_wp; i++)
{
frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
}

frenet_s += distance(0,0,proj_x,proj_y);

return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
int prev_wp = -1;

while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
{
prev_wp++;
}

int wp2 = (prev_wp+1)%maps_x.size();

double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
// the x,y,s along the segment
double seg_s = (s-maps_s[prev_wp]);

double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

double perp_heading = heading-pi()/2;

double x = seg_x + d*cos(perp_heading);
double y = seg_y + d*sin(perp_heading);

return {x,y};

}
////////////////////////////////////////End of the Udacity fonctions/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Initialization 
// the current vehicle is located at the beginning of the simulation on the middle line
int lane = 1; 
// the reference velocity is initializes with 0 mph (max : 50 mph)
double ref_vel = 0.0; 

int main() {
uWS::Hub h;

// Load up map values for waypoint's x,y,s and d normalized normal vectors
vector<double> map_waypoints_x;
vector<double> map_waypoints_y;
vector<double> map_waypoints_s;
vector<double> map_waypoints_dx;
vector<double> map_waypoints_dy;

// Waypoint map to read from
string map_file_ = "../data/highway_map.csv";
// The max s value before wrapping around the track back to 0
double max_s = 6945.554; // total length of the road

ifstream in_map_(map_file_.c_str(), ifstream::in);

string line;
while (getline(in_map_, line)) {
istringstream iss(line);
double x;
double y;
float s;
float d_x; // normal component in x axis for this way point
float d_y; // normal component in y axis for this way point
iss >> x;
iss >> y;
iss >> s;
iss >> d_x;
iss >> d_y;
map_waypoints_x.push_back(x);
map_waypoints_y.push_back(y);
map_waypoints_s.push_back(double(s));
map_waypoints_dx.push_back(double(d_x));
map_waypoints_dy.push_back(double(d_y));
}


h.onMessage([&map_waypoints_x,
			&map_waypoints_y,
			&map_waypoints_s,
			&map_waypoints_dx,
			&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
// "42" at the start of the message means there's a websocket message event.
// The 4 signifies a websocket message
// The 2 signifies a websocket event
// auto sdata = string(data).substr(0, length);
// cout << sdata << endl;
if (length && length > 2 && data[0] == '4' && data[1] == '2') {

auto s = hasData(data);

if (s != "") {
auto j = json::parse(s);

string event = j[0].get<string>();

if (event == "telemetry") {
// j[1] is the data JSON object

// Main car's localization Data
double car_x = j[1]["x"];
double car_y = j[1]["y"];
double car_s = j[1]["s"];
double car_d = j[1]["d"];
double car_yaw = j[1]["yaw"];
double car_speed = j[1]["speed"];

// Previous path data given to the Planner
auto previous_path_x = j[1]["previous_path_x"];
auto previous_path_y = j[1]["previous_path_y"];
int prev_size = previous_path_x.size();

// Previous path's end s and d values
double end_path_s = j[1]["end_path_s"];
double end_path_d = j[1]["end_path_d"];

// Sensor Fusion Data, a list of all other cars on the same side of the road.
auto sensor_fusion = j[1]["sensor_fusion"];

////////////////////////////////////////////PART 1= SENSOR FUSION//////////////////////////////////////////////////

// if the previous path in X from the planner >0, our current vehicle takes the previous path in longitudinale coordinate -S
if(prev_size >0)
car_s = end_path_s;

// definition, Bit Checking initialization
bool danger = false; // Bit checking distance between from our current vehicle to front vehicle
bool left_available = true; // Bit Cheking for a vehicle presence on the left lane
bool right_available = true; // Bit Cheking for a vehicle presence on the left lane
double safe_distance = 30.0; // safe distance to keep between our vehicle and the vehicle located on the front : 30 meters

// search all the visible vehicles from the sensor fusion
// store the data from the sensor fusion with this model : [id,x,y,vx,vy,s,d]
for(int i=0; i<sensor_fusion.size(); i++){

	double id = sensor_fusion[i][0]; // 1 specific Id for every vehicle
	double x = sensor_fusion[i][1]; // Global X-MAP coordinates
	double y = sensor_fusion[i][2]; // Global Y-MAP coordinates
	
	double vx = sensor_fusion[i][3]; // velocity in x to predict where will be the vehicle in the future
	double vy = sensor_fusion[i][4]; // velocity in y to predict where will be the vehicle in the future
	double speed = sqrt(vx*vx + vy*vy); 
	double s = sensor_fusion[i][5]; // local Frenet coordinate vehicle longitudinal
	double d = sensor_fusion[i][6]; // local Frenet coordinate vehicle latitude

// use of the previous points to predict the future s value
s += double(prev_size) * 0.02 * speed;

////////////////////////BEHAVIOR PLANNING : ACC = Adapative Cruise control///////////////////
///////////////////////Check the respect of safety distance with the vehicle front///////////

//(d>0 et d<4) return lane 0
//(d>4 et d<8) return lane 1
//(d>8 et d<12) return lane 2
if( d < (2+4*lane+2) && d > (2+4*lane-2) ){
	if( s > car_s && (s - car_s) < safe_distance ){
		danger = true; // safety distance not respected, consequently an overtake must be prepared
	}
}

//////////////////////BEHAVIOR PLANNING : Check lane LEFT to accept (or not)an overtake (Preparation)////
int left_lane = lane - 1;
// Condition to accept or not the change Left
if( left_available &&( d < (2+4*left_lane+2) && d > (2+4*left_lane-2) ) ){ 
	double delta_distance = abs(s - car_s); // the danger vehicle can be on the front or on the rear in the lane left
	if (delta_distance < safe_distance){
		left_available = false;
	}
	}
//////////////////////BEHAVIOR PLANNING : Check lane RIGHT to accept (or not)an overtake (Preparation)////
int right_lane = lane + 1;
// Condition to accept or not the change Right
if( right_available && ( d < (2+4*right_lane+2) && d > (2+4*right_lane-2) ) ){
	double delta_distance = abs(s - car_s); // // the danger vehicle can be on the front or on the rear in the lane right
	if (delta_distance < safe_distance){ 
		right_available = false;
	}
	}
}

/////////////////////BEHAVIOR PLANNING : LANE CHANGER (if PREPARATION STEP is validated)//////////////////////////////////

if(danger==1) // keep lane + slow down OR change Left or chanhe Right

if (lane >0 && left_available){ // change to the lane left
	lane -= 1;
	} 
else if (lane < 2 && right_available){ // change to the lane right
	lane += 1;
	} 
else{ // keep line slow down
	ref_vel -= 0.224; // - 5 m/s2 for the confort criteria
	}

// if no vehicle on the front and in the same lane, acceleration Until 50-0.5 = 49.5 MPH
else if(ref_vel < 49.5){ // maximum speed = 49.5 MPH
	ref_vel += 0.224; // + 5 m/s2 for the confort criteria
}

/////////////////////////////////////////////////SPLINE CREATION////////////////////
// create 5 points that the spline will use to generate the fonction 

// creation of set of points
vector<double> ptsx;
vector<double> ptsy;


// reference x, y, yaw states
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);


// if previous size is almost empty, use the car as starting reference
if(prev_size < 2){

	// use two points that make the path tangent to the car
	double prev_car_x = car_x - cos(car_yaw);
	double prev_car_y = car_y - sin(car_yaw);

	ptsx.push_back(prev_car_x);
	ptsx.push_back(car_x);
	ptsy.push_back(prev_car_y);
	ptsy.push_back(car_y);
}
// use last points from previous path
else{

	ref_x = previous_path_x[prev_size-1];
	ref_y = previous_path_y[prev_size-1];

	double ref_x_prev = previous_path_x[prev_size-2];
	double ref_y_prev = previous_path_y[prev_size-2];
	ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

	// use two points that make the path tangent to the previous path's end point
	ptsx.push_back(ref_x_prev);
	ptsx.push_back(ref_x);

	ptsy.push_back(ref_y_prev);
	ptsy.push_back(ref_y);
}


// in frenet add evenly 30 meter spaced points ahead of the starting reference
vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

// go from global to local coordinates , x=y=yaw=0
for(int i=0; i<ptsx.size(); i++){
	// shift car reference angle to 0 degree
	double shift_x = ptsx[i] - ref_x;
	double shift_y = ptsy[i] - ref_y;

	ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
	ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
}


// create a spline
tk::spline s;

// set(x,y) points to the spline
s.set_points(ptsx, ptsy);

////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////Waypoint Planner/////////////////////////////////
// Create each of the individuals points in x that the vehicle follows and feeds them into the spline fonction
vector<double> next_x_vals;
vector<double> next_y_vals;


// start with all of the previous path points from last time
for(int i=0; i<previous_path_x.size(); i++){
	next_x_vals.push_back(previous_path_x[i]);
	next_y_vals.push_back(previous_path_y[i]);
}

// calculate how to break up spline points so that we travel at our desired reference velocity
double target_x = 30; // path planner's farthest point is 30 meters ahead
double target_y = s(target_x);
double target_dist = sqrt( target_x * target_x + target_y * target_y);
double x_add_on = 0;

double N = target_dist / (0.02*ref_vel/2.24); // divide by 2.24 to transfer from miles/h to meters/s
double x_step = target_x / N;
//use the spline to look for the xy points
for (int i = 1; i <= 50 - previous_path_x.size(); i++){ // 50: defined total number of points

	double x_point = x_add_on + x_step;
	double y_point = s(x_point);

	x_add_on = x_point;

	double x_ref = x_point;
	double y_ref = y_point;

	// rotate back to world coordinate after rotating it earlier
	x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
	y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

	x_point += ref_x;
	y_point += ref_y;

	next_x_vals.push_back(x_point);
	next_y_vals.push_back(y_point);
}


json msgJson;
msgJson["next_x"] = next_x_vals;
msgJson["next_y"] = next_y_vals;

auto msg = "42[\"control\","+ msgJson.dump()+"]";

//this_thread::sleep_for(chrono::milliseconds(1000));
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

}
} else {
// Manual driving
std::string msg = "42[\"manual\",{}]";
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}
}
});

// We don't need this since we're not using HTTP but if it's removed the
// program
// doesn't compile :-(
h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
size_t, size_t) {
const std::string s = "<h1>Hello world!</h1>";
if (req.getUrl().valueLength == 1) {
res->end(s.data(), s.length());
} else {
// i guess this should be done more gracefully?
res->end(nullptr, 0);
}
});

h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
std::cout << "Connected!!!" << std::endl;
});

h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
char *message, size_t length) {
ws.close();
std::cout << "Disconnected" << std::endl;
});

int port = 4567;
if (h.listen(port)) {
std::cout << "Listening to port " << port << std::endl;
} else {
std::cerr << "Failed to listen to port" << std::endl;
return -1;
}
h.run();
}
