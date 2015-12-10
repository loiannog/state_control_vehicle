#ifndef NANO_KONTROL2
#define NANO_KONTROL2

#include <sensor_msgs/Joy.h>

// Buttons
static const int traj_button = 34; // Cycle
static const int estop_button = 26;  // Stop
static const int play_button = 27;  // Play
static const int motors_on_button = 28;  // Rec
static const int line_tracker_button = 29;  // Track L
static const int velocity_tracker_button = 30; // Track R
static const int hover_button = 31; // Marker Set
static const int line_tracker_yaw_button = 24; // Rewind 
static const int home_button = 0; // Leftmost "S" button
static const int land_button = 6;
static const int eland_button = 7; // Rightmost "S" button

struct buttons {
  bool rewind;
  bool fast_forward;
  bool stop;
  bool play;
  bool record;
  bool s[8];
  bool m[8];
  bool r[8];
  bool track_left;
  bool track_right;
  bool cycle;
  bool marker_set;
  bool marker_left;
  bool marker_right;
} buttons;

// updateButtons(const sensor_msgs::Joy::ConstPtr &msg, &buttons)
// {
//   // buttons.rewind = 
//   // buttons.fast_forward = msg->buttons[];
//   buttons.stop
//   buttons.cycle = msg->buttons[34];
// }

#endif
