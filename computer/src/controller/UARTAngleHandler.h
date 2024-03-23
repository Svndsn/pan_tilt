#include <cstdint>

class UARTAngleHandler {
public:
  // Constructor
  UARTAngleHandler();
  // Destructor
  ~UARTAngleHandler();

  // Method to receive the angle from the UART
  void receiveAngles();

  // Method to send the angle to the UART
  void sendAngles();

  // Method to get the pan angle
  int16_t getPanAngle();

  // Method to get the tilt angle
  int16_t getTiltAngle();

  // Method to set the pan angle
  void setPanAngle(int16_t targetAngle);

  // Method to set the tilt angle
  void setTiltAngle(int16_t targetAngle);

private:
  int16_t m_panAngle, m_tiltAngle;
};
