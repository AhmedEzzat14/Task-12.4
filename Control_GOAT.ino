/**
*****************************************************************************************
** Author  : Ahmed Ezzat
** Project : Control speed of motor and implement PID & Exponential Smoothing Filter 
** Date    : 9/8/2024
*****************************************************************************************
**/

#define ENABLE 9                               // Connect Enable pin of LM295 with Arduino's pin 9 (PWM)
#define INT1   2                              // Connect Int1 pin of LM295 with Arduino's pin 2
#define INT2   3                             // Connect Int2 pin of LM295 with Arduino's pin 3
#define FEEDBACK_PIN A0                     // Pin for reading motor speed feedback (e.g., from an encoder or potentiometer)

double Alpha = 0.2;                       // Smoothing factor for Exponential Smoothing Filter
double Smoothed_Value = 0;               // Filtered value

double Set_Point = 150;                // Desired motor speed (PWM value 0-255)


class PID_Controller {              // PID Controller class
public:
    PID_Controller(double kp, double ki, double kd) 
        : Kp(kp), Ki(ki), Kd(kd), Integral(0), Previous_Error(0), Last_Time(millis()) {}

    double Compute(double setPoint, double actualValue){
        double Time_Now = millis();
        double dt = (Time_Now - Last_Time) / 1000.0;                                  // Time difference in seconds
        Last_Time = Time_Now;

        if(dt == 0){
        Serial.print("Error: dt is zero");
        return 0;
      }

        double Error = setPoint - actualValue;                                    // Calculate error

        Integral += Error * dt;                                                 // Update Integral term
        double Derivative = (Error - Previous_Error) / dt;                     // Calculate Derivative term
        Previous_Error = Error;                                               // Store Error for next iteration

        double Output = (Kp * Error) + (Ki * Integral) + (Kd * Derivative); // Compute PID output
        return constrain(Output, 0, 255);                                  // Constrain output to valid PWM range
    }

private:
    double Kp;                 // Proportional coefficient
    double Ki;                // Integral coefficient
    double Kd;               // Derivative coefficient
    double Integral;        // Integral term
    double Previous_Error; // Previous error value
    double Last_Time;     // Last time PID compute was called
};

PID_Controller PID_ME(0.8, 20, 0.001);                                            // Initialize PID Controller with specific coefficients

void setup() {
    Serial.begin(115200);                                                      // Initialize serial communication for debugging

    pinMode(ENABLE, OUTPUT);                                                 // Set motor control pins as output
    pinMode(INT1, OUTPUT);
    pinMode(INT2, OUTPUT);

    digitalWrite(INT1, LOW);                                              // Set motor direction forward (INT1 low, INT2 high)
    digitalWrite(INT2, HIGH);

    analogWrite(ENABLE, 0);                                            // Initialize PWM value to 0

    Smoothed_Value = map(analogRead(FEEDBACK_PIN), 0, 1023, 0, 255); // Scaling smoothing value
}

void loop() {
    double Actual_Value = map(analogRead(FEEDBACK_PIN), 0, 1023, 0, 255);         // Read and scale feedback value

    Smoothed_Value = ExponentialSmoothing(Actual_Value, Smoothed_Value, Alpha); // Apply exponential smoothing

    double OutPut = PID_ME.Compute(Set_Point, Smoothed_Value);                // Compute PID output

    analogWrite(ENABLE, (int)OutPut);                                       // Set motor speed using PWM

    // Print debug information to serial monitor
    Serial.print("Set Point: "); Serial.print(Set_Point);
    Serial.print("\tActual Value: "); Serial.print(Smoothed_Value);
    Serial.print("\tOutput (PWM): "); Serial.println(OutPut);

    delay(1000);                                                    // Delay for stability
}

double ExponentialSmoothing(double Input, double Previous_Smoothed, double Alpha) {
    return Alpha * Input + (1 - Alpha) * Previous_Smoothed; // Apply exponential smoothing formula
}
