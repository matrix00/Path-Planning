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
#define min(x,y) (x<y? x:y)
#define max(x,y) (x>y? x:y)

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double MAX_SPEED=49.5;
double inc = 2.24;
double acc = 9.5; //acceleration max 10 m/s2
double dcc = 9.5; //decceleration max 10 m/s2
enum STATE {STATE_CONTINUE=0, STATE_SLOW_DOWN, STATE_PREPARE_LANE_CHANGE};

struct FAState {
	STATE currState;
	int currLane;
	int changeLaneTo;
}; 

FAState fas;

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
		prev_wp  = maps_x.size()-1;
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


bool evalLaneToSwitch(nlohmann::basic_json<> sensor_fusion, int lane, double car_s, double car_speed, int prev_size)
{
	bool canSwitch = true;

	for (int i=0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		cout << i << " ######### lane # of this car " << (int) d/4 << endl;
		if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane -2))
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx+vy*vy);
			double check_car_s = sensor_fusion[i][5];

			check_car_s -= ((double) prev_size * 0.02 * check_speed);
			//check_car_s -= ((double) 1 * fabs(car_speed+check_speed)); //distance cover by 1 sec

			//check s values greater than mine and s gap
//			if ((check_car_s > car_s && (check_car_s - car_s) < 30))
			cout << i<<  " $$$$$$$$$$$$$$$$$  SWITCH $$$$$$$$$$$$$$$$$$$$$$$$??? ------------------>" << car_s << " check car s" << check_car_s << " diff " << (car_s - check_car_s) << " lane " << (int) d/4 << endl;
			if (fabs(car_s - check_car_s) < 35)
			{
				canSwitch = false;
				cout << i<< " ************  DON'T SWITCH ************************* ------------------> "<< (car_s - check_car_s) <<endl;
							
			}
		}
	}

	if (canSwitch)
		cout << "################ SWITCHinG to  ************************* ------------------> " << lane << endl;
	else
		cout << "###############DON'T SWITCH ************************* ------------------> "<< lane  <<endl;

	
	return canSwitch;
}


int main() {
  uWS::Hub h;

	fas.currState = STATE_CONTINUE;
	fas.changeLaneTo = -1; //invalid


  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
		std::cout << "start Telemetry ";

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
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;
		int lane;
	
		double ref_val = car_speed;
		double prev_size=previous_path_x.size();
		std::cout << "prev size "<< prev_size<<endl;
		if (fas.changeLaneTo != -1) // need to change lane
		{
			fas.currLane = fas.changeLaneTo;
			fas.currState = STATE_CONTINUE;
		}
		else 
		{
			fas.currLane =  (int) (end_path_d/4);
			fas.changeLaneTo = -1;
		}
		lane = fas.currLane ;

		std::cout << "prev path x= "<< car_x << " y=" << car_y << " s=" << car_s << " d=" << car_d<< " yaw=" << car_yaw << " speed=" << car_speed<< " lane "<< lane << " state " << fas.currState << endl;
		if (prev_size >0)
		{
			car_s = end_path_s;
		}
		bool too_close = false;

		cout << "Sensor fusion size " << sensor_fusion.size() << endl;
		//find ref_v to use

		//if (fas.currState != STATE_PREPARE_LANE_CHANGE )
		//{
		for (int i=0; i < sensor_fusion.size(); i++)
		{
			//find car in my lane
			float d = sensor_fusion[i][6];
			if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane -2))
			{
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double check_speed = sqrt(vx*vx+vy*vy);
				double check_car_s = sensor_fusion[i][5];

				check_car_s += ((double) prev_size * 0.02 * check_speed);
				//check s values greater than mine and s gap
				if ((check_car_s > car_s && (check_car_s - car_s) < 30))
				{
					//TODO: 
					too_close = true;
					//change to a lane
					//if too close, try different lanes
					bool canSwitch = false;
					switch (lane)
					{
						case 0:
						case 2:
							canSwitch = evalLaneToSwitch(sensor_fusion, 1, car_s, car_speed, prev_size);
							if (canSwitch)
							{
								lane = 1;
								cout << "Yayyyyy   SWITCH to************************* ------------------> "<< lane  <<endl;
								fas.currState = STATE_PREPARE_LANE_CHANGE;
								fas.changeLaneTo = 1;
							}
							else
							{
								//reduce speed more, can't change lane
								//ref_val -= 5;
								cout << "CANTTTTTTTTTT   SWITCH to************************* ------------------> reduce speed to "<< ref_val  <<endl;
								fas.currState = STATE_SLOW_DOWN;
							}

							break;

						case 1:
							canSwitch = evalLaneToSwitch(sensor_fusion, 0, car_s, car_speed, prev_size);
							if (canSwitch)
							{
								//lane = 0;
								cout << "Yayyyyy   SWITCH to************************* ------------------> "<< lane  <<endl;
								fas.currState = STATE_PREPARE_LANE_CHANGE;
								fas.changeLaneTo = 0;
							}
							else if (evalLaneToSwitch(sensor_fusion, 2, car_s, car_speed, prev_size))
							{
								//lane = 2;
								cout << "Yayyyyy   SWITCH to************************* ------------------> "<< lane  <<endl;
								fas.currState = STATE_PREPARE_LANE_CHANGE;
								fas.changeLaneTo = 2;
							}
							else
							{
								//reduce speed more, can't change lane
								//ref_val -= 5;
								cout << "CANTTTTTTTTTT   SWITCH to************************* ------------------> reduce speed to "<< ref_val  <<endl;
								fas.currState = STATE_SLOW_DOWN;
							}

					}
				}
			}
			
		}

		//}


		if (fas.currState == STATE_SLOW_DOWN)
		{
			if (!too_close)
				fas.currState = STATE_CONTINUE;
			else
			{
				//v = u  - a t
//				ref_val = max(ref_val - inc, 10);
				ref_val = max(ref_val - dcc * 0.02 * 2.24, 10);
				cout << "---------reducing speed, new speed " << ref_val << endl;
			}
		}
		else if (ref_val < MAX_SPEED-0.5)
		{
//			ref_val = min(ref_val+inc, MAX_SPEED);
//			v = u  + a t
			ref_val = min(ref_val +  acc * 0.5 * 2.24, MAX_SPEED);
			cout << "+++++++++ increasing speed, new speed " << ref_val << endl;
		}		

		
		cout<<"*** adj current speed is "<< ref_val << " current lane " << lane << endl;

		vector<double> ptsx;
		vector<double> ptsy;

		//reference x, y, yaw states
		//
		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		//if previous state is almost empty, use the car as start
		if (prev_size <2)
		{
			std::cout << "prev size less than 2 " << prev_size << endl;
			//use two points that make the path tangent to the car
			double prev_car_x = car_x - cos(car_yaw);
			double prev_car_y = car_y - sin(car_yaw);

			ptsx.push_back(prev_car_x);
			ptsx.push_back(car_x);

			ptsy.push_back(prev_car_y);
			ptsy.push_back(car_y);
		}
		else //use the previous path's end point as starting reference.
		{
			std::cout << "prev size not < 2 " << prev_size << " ptsx size "<< ptsx.size()<< endl;
			ref_x = previous_path_x[prev_size -1];
			ref_y = previous_path_y[prev_size -1];

			double ref_x_prev = previous_path_x[prev_size - 2];
			double ref_y_prev = previous_path_y[prev_size - 2];
			ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

			//use the two points that make the path tangent to the previous path's end

			ptsx.push_back(ref_x_prev);
			ptsx.push_back(ref_x);

			ptsy.push_back(ref_y_prev);
			ptsy.push_back(ref_y);
			
		}

		//cout<< " prev points" << "ptsx x="<< ptsx[0] << "," << ptsx[1] << "ptsy  y=" << ptsy[0] << "," << ptsy[1]<< " ref x" << ref_x << " refy " << ref_y<< " ptsx size " << ptsx.size()<< endl;

		//In Frenet add evenly 30m spaced points ahead of the starting ref
		vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);
			
		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);

		//cout<< "wp x" << next_wp0[0] << " wp1 " << next_wp1[0] << " wp2 " << next_wp2[0]<< endl; 
		//cout<< "wp y" << next_wp0[1] << " wp1 " << next_wp1[1] << " wp2 " << next_wp2[1]<< endl; 

		//cout<< "ref points x="<< ref_x << " y=" << ref_y << " Yaw=" << ref_yaw << " ptsx size " << ptsx.size()<<endl;
		for (int i=0; i < ptsx.size(); i++)
		{
			//cout<< " i=" << i << "ptsx x="<< ptsx[i] << "ptsy  y=" << ptsy[i] << " ref x" << ref_x << " refy " << ref_y<<endl;
			//shift car ref angle to 0 degrees
			double shift_x = ptsx[i] - ref_x;
			double shift_y = ptsy[i] - ref_y;

			ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
			ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

			//cout<< "i " << i << " x " << ptsx[i] << " y " << ptsy[i]<< endl; 
		}
	
		//create a spline
		tk::spline s;

		//set (x,y) point to the spline
		s.set_points(ptsx, ptsy);

		//define the actual (x,y) point that will be used for the planner
		vector<double> next_x_vals;
          	vector<double> next_y_vals;

		//start with all of the previous path points from the last time
		for (int i=0; i < previous_path_x.size(); i++)
		{
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}

		//calculate how to break the spline points so that we travel at our ref velocity
		double target_x = 30.0;
		double target_y = s(target_x);

		double target_dist = sqrt(target_x * target_x + target_y * target_y);


		double x_add_on = 0;

		int total_points = 50;
		double N = (target_dist/(0.02 * ref_val/2.24));

		cout << "Target dist "<< target_dist << " target y " << target_y << " N " << N << endl;

		//fill the rest of the path planner after using previous points
		for (int i=0; i < total_points - previous_path_x.size(); i++)
		{
			//Sample every 0.02 sec. 
			double x_point = x_add_on + (target_x)/N;
			double y_point = s(x_point);
			
			x_add_on = x_point;

			double x_ref = x_point;
			double y_ref = y_point;

			//rotate back to normal 
			x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
			y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

			x_point  += ref_x;
			y_point  += ref_y;

			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);
		}

		//std::cout<< "x_vals size " << next_x_vals.size() << " y_vals size " << next_y_vals.size()<< "\n";
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
