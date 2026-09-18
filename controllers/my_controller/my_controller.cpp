#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <iostream>
#include <cmath>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  Motor *motor = robot->getMotor("MOTOR");
  PositionSensor *encoder = robot->getPositionSensor("ENCODER");
  Keyboard *kb = robot->getKeyboard();

  encoder->enable(timeStep);
  kb->enable(timeStep);

  motor->setPosition(INFINITY);
  motor->setVelocity(0.0);

  double Kp = 15.0;
  double Ki = 0.0;
  double Kd = 0.5;

  double setpoint = 0.0;
  double previous_error = 0.0;
  double integral = 0.0;

  cout << "Controlador PID iniciado." << endl;

  while (robot->step(timeStep) != -1) {

    // Leer posición angular
    double current_position = encoder->getValue();

    // Calcular error
    double error = setpoint - current_position;

    // Normalizar error angular a [-PI, PI]
    while (error > M_PI)
      error -= 2.0 * M_PI;

    while (error < -M_PI)
      error += 2.0 * M_PI;

    // Tiempo
    double dt = timeStep / 1000.0;

    // Integral
    integral += error * dt;

    // Derivada
    double derivative = (error - previous_error) / dt;

    // PID
    double output_velocity =
        (Kp * error) +
        (Ki * integral) +
        (Kd * derivative);

    // Perturbación manual
    int key = kb->getKey();

    if (key == 'A' || key == 'a') {
      output_velocity = 10.0;
    }

    // Limitar velocidad
    if (output_velocity > 10.0)
      output_velocity = 10.0;

    if (output_velocity < -10.0)
      output_velocity = -10.0;

    motor->setVelocity(output_velocity);

    previous_error = error;
  }

  delete robot;
  return 0;
}