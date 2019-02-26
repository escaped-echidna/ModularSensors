/*
 *VariableBase.h
 *This file is part of the EnviroDIY modular sensors library for Arduino
 *
 *Initial library developement done by Sara Damiano (sdamiano@stroudcenter.org).
 *
 *This file is for the variable base class.
*/

// Header Guards
#ifndef VariableBase_h
#define VariableBase_h

// Debugging Statement
// #define DEBUGGING_SERIAL_OUTPUT Serial

// Forward Declared Dependences
class Sensor;

// Included Dependencies
#include "ModSensorDebugger.h"

class Variable
{
public:
    // The constructor for a measured variable - that is, one whose values are
    // updated by a sensor.  This can have arguments because they will all be
    // called in the initializer list from the header file for the sensor type
    // that the variable will be tied to.
    Variable(const uint8_t sensorVarNum,
             uint8_t decimalResolution,
             const char *varName,
             const char *varUnit,
             const char *varCode);

     // The constructor for a calculated variable - that is, one whose value is
     // calculated by the calcFxn which returns a float.  All arguments for this
     // have to be set in the begin function.
     Variable();

    // Destructor
    virtual ~Variable();

    // This does all of the setup that can't happen in the constructors
    // That is, anything that is dependent on another object having been created
    // first or anything that requires the actual processor/MCU to do something.
    Variable *begin(Sensor *parentSense, const char *uuid,
                    const char *customVarCode);
    Variable *begin(Sensor *parentSense, const char *uuid);
    Variable *begin(Sensor *parentSense);
    Variable *begin(float (*calcFxn)(),
                    uint8_t decimalResolution,
                    const char *varName,
                    const char *varUnit,
                    const char *varCode,
                    const char *uuid);
    Variable *begin(float (*calcFxn)(),
                    uint8_t decimalResolution,
                    const char *varName,
                    const char *varUnit,
                    const char *varCode);

    // These functions tie the variable and sensor together
    // They should never be called for a calculated variable
    // This notifies the parent sensor that it has an observing variable
    void attachSensor(Sensor *parentSense);
    // This is the function called by the parent sensor's notifyVariables() function
    virtual void onSensorUpdate(Sensor *parentSense);
    // This is a helper - it returns the name of the parent sensor, if applicable
    // This is needed for dealing with variables in arrays
    String getParentSensorName(void);
    // This is a helper - it returns the name and location of the parent sensor, if applicable
    // This is needed for dealing with variables in arrays
    String getParentSensorNameAndLocation(void);

    // This ties a calculated variable to its calculation function
    void setCalculation(float (*calcFxn)());

    // This sets up the variable (generally attaching it to its parent)
    // virtual bool setup(void);

    // This gets the variable's name using http://vocabulary.odm2.org/variablename/
    String getVarName(void);
    // This gets the variable's unit using http://vocabulary.odm2.org/units/
    String getVarUnit(void);
    // This returns a customized code for the variable, if one is given, and a default if not
    String getVarCode(void);
    // This returns the variable UUID, if one has been assigned
    String getVarUUID(void);

    // This sets the variable code to a new custom value
    void setVarCode(const char *varCode);
    // This sets the UUID
    void setVarUUID(const char *uuid);

    // This returns the current value of the variable as a float
    float getValue(bool updateValue = false);
    // This returns the current value of the variable as a string with the
    // correct number of significant figures
    String getValueString(bool updateValue = false);

    // This is the parent sensor for the variable
    Sensor *parentSensor;
    bool isCalculated;

protected:
    float _currentValue;

private:
    float (*_calcFxn)(void);

    const uint8_t _sensorVarNum;
    uint8_t _decimalResolution;

    const char *_varName;
    const char *_varUnit;
    const char *_varCode;
    const char *_uuid;
};

#endif  // Header Guard
