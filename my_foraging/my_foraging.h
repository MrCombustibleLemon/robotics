#ifndef MY_FORAGING_H
#define MY_FORAGING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class MyForaging : public CCI_Controller {

public:

   /*
    * This structure holds data about food collecting by the robots
    */
   struct SFoodData {
      bool HasFoodItem;      // true when the robot is carrying a food item

      SFoodData();
      void Reset();
   };

   /*
    * The following variables are used as parameters for the
    * diffusion algorithm. You can set their value in the <parameters>
    * section of the XML configuration file, under the
    * <controllers><my_foraging_controller><parameters><diffusion>
    * section.
    */
   struct SDiffusionParams {
      /*
       * Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;

      /* Constructor */
      SDiffusionParams();

      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode& t_tree);
   };

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><my_foraging_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The three possible states in which the controller can be */
      enum EState {
         STATE_RESTING = 0,
         STATE_EXPLORING,
         STATE_RETURN_TO_NEST
      } State;

      /* True when the robot is in the nest */
      bool InNest;
      SStateData();
      void Reset();
   };

public:

   /* Class constructor. */
   MyForaging();
   /* Class destructor. */
   virtual ~MyForaging() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><my_foraging_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /*
    * Returns true if the robot is currently exploring.
    */
   inline bool IsExploring() const {
      return m_sStateData.State == SStateData::STATE_EXPLORING;
   }

   /*
    * Returns true if the robot is currently resting.
    */
   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_RESTING;
   }

   /*
    * Returns true if the robot is currently returning to the nest.
    */
   inline bool IsReturningToNest() const {
      return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
   }

   /*
    * Returns the food data
    */
   inline SFoodData& GetFoodData() {
      return m_sFoodData;
   }

private:

   /*
    * Updates the state information.
    * In pratice, it sets the SStateData::InNest flag.
    * Future, more complex implementations should add their
    * state update code here.
    */
   void UpdateState();

   /*
    * Calculates the vector to the light. Used to perform
    * phototaxis and antiphototaxis.
    */
   CVector2 CalculateVectorToLight();

   /*
    * Calculates the diffusion vector. If there is a close obstacle,
    * it points away from it; it there is none, it points forwards.
    * The b_collision parameter is used to return true or false whether
    * a collision avoidance just happened or not. It is necessary for the
    * collision rule.
    */
   CVector2 DiffusionVector(bool& b_collision);

   /*
    * Gets a direction vector as input and transforms it into wheel
    * actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   /*
    * Executes the resting state.
    */
   void Rest();

   /*
    * Executes the exploring state.
    */
   void Explore();

   /*
    * Executes the return to nest state.
    */
   void ReturnToNest();

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the foot-bot motor ground sensor */
   CCI_FootBotMotorGroundSensor* m_pcGround;

   /* The controller state information */
   SStateData m_sStateData;
   /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;
   /* The food data */
   SFoodData m_sFoodData;

};

#endif

