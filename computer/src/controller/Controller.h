#include <utility>
class Controller {
public:
  Controller();
  ~Controller();

  // Get the current values of the left joystick from -1 to 1
  std::pair<float, float> getAxis();

  // Get state of X button
  bool isXButtonPressed();

  // Get state of triangle button
  bool isTriangleButtonPressed();

  // Is the controller connected
  bool isConnected();

  // Try to reconnect the controller
  bool reconnect();

private:
};
