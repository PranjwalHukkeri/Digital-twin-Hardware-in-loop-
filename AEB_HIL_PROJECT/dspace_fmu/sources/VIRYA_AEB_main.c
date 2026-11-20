#include <math.h>
#include <stddef.h>

#define FMI2_FUNCTION_PREFIX VIRYA_AEB_


#include "fmi2Functions.h"


/* ============================================================
   VIRYA AEB - Validated Parameters
   ============================================================ */

#define TTC_WARNING   4.00
#define TTC_LEVEL_1   3.00
#define TTC_LEVEL_2   2.25
#define TTC_LEVEL_3   1.75

#define A_LEVEL_1    -2.0
#define A_LEVEL_2    -4.0
#define A_LEVEL_3    -6.0

#define DT             0.05
#define MIN_BRAKE_DURATION 0.20

/* FMI GUID - MUST MATCH modelDescription.xml */
#define FMU_IMPLEMENTATION_GUID \
"{7f3c9b21-6a42-4e85-91d7-virya0000001}"


/* ============================================================
   AEB States
   ============================================================ */

#define STATE_SAFE       0
#define STATE_WARNING    1
#define STATE_LEVEL_1    2
#define STATE_LEVEL_2    3
#define STATE_LEVEL_3    4


/* ============================================================
   FMU Instance
   ============================================================ */

typedef struct
{
    int current_state;
    double state_time;

    /* Inputs */
    double ego_speed;
    double target_speed;
    double distance;

    /* Outputs */
    double relative_velocity;
    double TTC;
    int AEB_state;
    double desired_deceleration;

} VIRYA_AEB_Instance;


/* ============================================================
   FMI Instantiate
   ============================================================ */

FMI2_Export fmi2Component fmi2Instantiate(
    fmi2String instanceName,
    fmi2Type fmuType,
    fmi2String fmuGUID,
    fmi2String fmuResourceLocation,
    const fmi2CallbackFunctions* functions,
    fmi2Boolean visible,
    fmi2Boolean loggingOn)
{
    VIRYA_AEB_Instance* instance;

    (void)instanceName;
    (void)fmuResourceLocation;
    (void)visible;
    (void)loggingOn;

    if (fmuType != fmi2CoSimulation)
        return NULL;

    if (fmuGUID == NULL)
        return NULL;

    if (functions == NULL)
        return NULL;

    instance = (VIRYA_AEB_Instance*)
        functions->allocateMemory(1, sizeof(VIRYA_AEB_Instance));

    if (instance == NULL)
        return NULL;

instance->current_state = STATE_SAFE;
instance->state_time = 0.0;

instance->ego_speed = 1.0;
instance->target_speed = 0.0;
instance->distance = 5.0;

instance->relative_velocity = 1.0;
instance->TTC = 5.0;
instance->AEB_state = STATE_SAFE;
instance->desired_deceleration = 0.0;
    return (fmi2Component)instance;
}


/* ============================================================
   FMI Free Instance
   ============================================================ */

FMI2_Export void fmi2FreeInstance(
    fmi2Component c)
{
    /* Memory is released by the dSPACE FMI environment. */
    (void)c;
}


/* ============================================================
   FMI Setup Experiment
   ============================================================ */

FMI2_Export fmi2Status fmi2SetupExperiment(
    fmi2Component c,
    fmi2Boolean toleranceDefined,
    fmi2Real tolerance,
    fmi2Real startTime,
    fmi2Boolean stopTimeDefined,
    fmi2Real stopTime)
{
    (void)c;
    (void)toleranceDefined;
    (void)tolerance;
    (void)stopTimeDefined;
    (void)stopTime;

    if (startTime != 0.0)
        return fmi2Error;

    return fmi2OK;
}


/* ============================================================
   FMI Enter Initialization Mode
   ============================================================ */

FMI2_Export fmi2Status fmi2EnterInitializationMode(
    fmi2Component c)
{
    (void)c;
    return fmi2OK;
}


/* ============================================================
   FMI Exit Initialization Mode
   ============================================================ */


FMI2_Export fmi2Status fmi2ExitInitializationMode(
    fmi2Component c)
{
    VIRYA_AEB_Instance* instance =
        (VIRYA_AEB_Instance*)c;

    if (instance == NULL)
        return fmi2Error;

    instance->current_state = STATE_SAFE;
    instance->state_time = 0.0;

    instance->relative_velocity =
        instance->ego_speed - instance->target_speed;

    if (instance->relative_velocity > 0.0 &&
        instance->distance > 0.0)
    {
        instance->TTC =
            instance->distance / instance->relative_velocity;
    }
    else
    {
        instance->TTC = 999.0;
    }

    instance->AEB_state = STATE_SAFE;
    instance->desired_deceleration = 0.0;

    return fmi2OK;
}

/* ============================================================
   FMI Terminate
   ============================================================ */

FMI2_Export fmi2Status fmi2Terminate(
    fmi2Component c)
{
    (void)c;
    return fmi2OK;
}


/* ============================================================
   FMI Reset
   ============================================================ */


FMI2_Export fmi2Status fmi2Reset(
    fmi2Component c)
{
    VIRYA_AEB_Instance* instance =
        (VIRYA_AEB_Instance*)c;

    if (instance == NULL)
        return fmi2Error;

    instance->current_state = STATE_SAFE;
    instance->state_time = 0.0;

    instance->ego_speed = 1.0;
    instance->target_speed = 0.0;
    instance->distance = 5.0;

    instance->relative_velocity = 1.0;
    instance->TTC = 5.0;
    instance->AEB_state = STATE_SAFE;
    instance->desired_deceleration = 0.0;

    return fmi2OK;
}


/* ============================================================
   FMI Get Real
   ============================================================ */


FMI2_Export fmi2Status fmi2GetReal(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    fmi2Real value[])
{
    VIRYA_AEB_Instance* instance =
        (VIRYA_AEB_Instance*)c;
    size_t i;

    if (instance == NULL || (nvr > 0 && (vr == NULL || value == NULL)))
        return fmi2Error;

    for (i = 0; i < nvr; i++)
    {
        switch (vr[i])
        {
        case 2000:
            value[i] = instance->relative_velocity;
            break;
        case 2001:
            value[i] = instance->TTC;
            break;
        case 2003:
            value[i] = instance->desired_deceleration;
            break;
        default:
            return fmi2Error;
        }
    }

    return fmi2OK;
}

/* ============================================================
   FMI Set Real
   ============================================================ */


FMI2_Export fmi2Status fmi2SetReal(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    const fmi2Real value[])
{
    VIRYA_AEB_Instance* instance =
        (VIRYA_AEB_Instance*)c;
    size_t i;

    if (instance == NULL || vr == NULL || value == NULL)
        return fmi2Error;

    for (i = 0; i < nvr; i++)
    {
        switch (vr[i])
        {
        case 1000:
            instance->ego_speed = value[i];
            break;

        case 1001:
            instance->target_speed = value[i];
            break;

        case 1002:
            instance->distance = value[i];
            break;

        default:
            return fmi2Error;
        }
    }

    return fmi2OK;
}



/* ============================================================
   FMI Get Integer
   ============================================================ */

FMI2_Export fmi2Status fmi2GetInteger(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    fmi2Integer value[])
{
    VIRYA_AEB_Instance* instance =
        (VIRYA_AEB_Instance*)c;
    size_t i;

    if (instance == NULL ||
        (nvr > 0 && (vr == NULL || value == NULL)))
        return fmi2Error;

    for (i = 0; i < nvr; i++)
    {
        if (vr[i] == 2002)
            value[i] = (fmi2Integer)instance->AEB_state;
        else
            return fmi2Error;
    }

    return fmi2OK;
}


/* ============================================================
   FMI Set Integer
   ============================================================ */

FMI2_Export fmi2Status fmi2SetInteger(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    const fmi2Integer value[])
{
    (void)c;
    (void)vr;
    (void)nvr;
    (void)value;

    /* This FMU has no writable Integer variables. */
    return fmi2Error;
}


/* ============================================================
   AEB FSM
   ============================================================ */

static int update_aeb_state(
    VIRYA_AEB_Instance* instance,
    double ttc,
    double v_rel)
{
    int state = instance->current_state;

    instance->state_time += DT;

    /* Safety-critical release */
    if (v_rel <= 0.0)
    {
        if (state != STATE_SAFE)
        {
            instance->current_state = STATE_SAFE;
            instance->state_time = 0.0;
        }

        return instance->current_state;
    }

    /* SAFE */
    if (state == STATE_SAFE)
    {
        if (ttc <= TTC_LEVEL_3)
        {
            instance->current_state = STATE_LEVEL_3;
            instance->state_time = 0.0;
        }
        else if (ttc <= TTC_LEVEL_2)
        {
            instance->current_state = STATE_LEVEL_2;
            instance->state_time = 0.0;
        }
        else if (ttc <= TTC_LEVEL_1)
        {
            instance->current_state = STATE_LEVEL_1;
            instance->state_time = 0.0;
        }
        else if (ttc <= TTC_WARNING)
        {
            instance->current_state = STATE_WARNING;
            instance->state_time = 0.0;
        }
    }

    /* WARNING */
    else if (state == STATE_WARNING)
    {
        if (ttc <= TTC_LEVEL_3)
        {
            instance->current_state = STATE_LEVEL_3;
            instance->state_time = 0.0;
        }
        else if (ttc <= TTC_LEVEL_2)
        {
            instance->current_state = STATE_LEVEL_2;
            instance->state_time = 0.0;
        }
        else if (ttc <= TTC_LEVEL_1)
        {
            instance->current_state = STATE_LEVEL_1;
            instance->state_time = 0.0;
        }
        else if (ttc > TTC_WARNING)
        {
            instance->current_state = STATE_SAFE;
            instance->state_time = 0.0;
        }
    }

    /* LEVEL 1 */
    else if (state == STATE_LEVEL_1)
    {
        if (ttc <= TTC_LEVEL_3)
        {
            instance->current_state = STATE_LEVEL_3;
            instance->state_time = 0.0;
        }
        else if (ttc <= TTC_LEVEL_2)
        {
            instance->current_state = STATE_LEVEL_2;
            instance->state_time = 0.0;
        }
        else if (instance->state_time >= MIN_BRAKE_DURATION)
        {
            if (ttc > TTC_LEVEL_1)
            {
                if (ttc <= TTC_WARNING)
                    instance->current_state = STATE_WARNING;
                else
                    instance->current_state = STATE_SAFE;

                instance->state_time = 0.0;
            }
        }
    }

    /* LEVEL 2 */
    else if (state == STATE_LEVEL_2)
    {
        if (ttc <= TTC_LEVEL_3)
        {
            instance->current_state = STATE_LEVEL_3;
            instance->state_time = 0.0;
        }
        else if (instance->state_time >= MIN_BRAKE_DURATION)
        {
            if (ttc > TTC_LEVEL_2)
            {
                if (ttc <= TTC_LEVEL_1)
                    instance->current_state = STATE_LEVEL_1;
                else if (ttc <= TTC_WARNING)
                    instance->current_state = STATE_WARNING;
                else
                    instance->current_state = STATE_SAFE;

                instance->state_time = 0.0;
            }
        }
    }

    /* LEVEL 3 */
    else if (state == STATE_LEVEL_3)
    {
        if (instance->state_time >= MIN_BRAKE_DURATION)
        {
            if (ttc > TTC_LEVEL_3)
            {
                if (ttc <= TTC_LEVEL_2)
                    instance->current_state = STATE_LEVEL_2;
                else if (ttc <= TTC_LEVEL_1)
                    instance->current_state = STATE_LEVEL_1;
                else if (ttc <= TTC_WARNING)
                    instance->current_state = STATE_WARNING;
                else
                    instance->current_state = STATE_SAFE;

                instance->state_time = 0.0;
            }
        }
    }

    return instance->current_state;
}




/* ============================================================
   FMI Do Step
   ============================================================ */

FMI2_Export fmi2Status fmi2DoStep(
    fmi2Component c,
    fmi2Real currentCommunicationPoint,
    fmi2Real communicationStepSize,
    fmi2Boolean noSetFMUStatePriorToCurrentPoint)
{
    VIRYA_AEB_Instance* instance =
        (VIRYA_AEB_Instance*)c;

    double v_rel;
    double ttc;
    int state;

    (void)currentCommunicationPoint;
    (void)noSetFMUStatePriorToCurrentPoint;

    /* Validate instance and communication step */
    if (instance == NULL || communicationStepSize <= 0.0)
        return fmi2Error;

    /* Calculate relative velocity */
    v_rel = instance->ego_speed - instance->target_speed;
    instance->relative_velocity = v_rel;

    /* Calculate time to collision */
    if (v_rel > 0.0 && instance->distance > 0.0)
    {
        ttc = instance->distance / v_rel;
    }
    else
    {
        ttc = 999.0;
    }

    instance->TTC = ttc;

    /* Update AEB finite state machine */
    state = update_aeb_state(instance, ttc, v_rel);
    instance->AEB_state = state;

    /* Set desired deceleration based on AEB state */
    switch (state)
    {
        case STATE_LEVEL_1:
            instance->desired_deceleration = A_LEVEL_1;
            break;

        case STATE_LEVEL_2:
            instance->desired_deceleration = A_LEVEL_2;
            break;

        case STATE_LEVEL_3:
            instance->desired_deceleration = A_LEVEL_3;
            break;

        default:
            instance->desired_deceleration = 0.0;
            break;
    }

    return fmi2OK;
}
/* ============================================================
   FMI Get/Set FMU State
   ============================================================ */

FMI2_Export fmi2Status fmi2GetFMUstate(
    fmi2Component c,
    fmi2FMUstate* FMUstate)
{
    (void)c;
    (void)FMUstate;

    return fmi2Error;
}


FMI2_Export fmi2Status fmi2SetFMUstate(
    fmi2Component c,
    fmi2FMUstate FMUstate)
{
    (void)c;
    (void)FMUstate;

    return fmi2Error;
}


/* ============================================================
   FMI Set Debug Logging
   ============================================================ */



FMI2_Export fmi2Status fmi2SetDebugLogging(
    fmi2Component c,
    fmi2Boolean loggingOn,
    size_t nCategories,
    const fmi2String categories[])
{
    (void)c;
    (void)loggingOn;
    (void)nCategories;
    (void)categories;

    return fmi2OK;
}



/* ============================================================
   End of VIRYA AEB FMU
   ============================================================ */