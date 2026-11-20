/* ============================================================
   VIRYA AEB - FMI 2.0 Co-Simulation FMU (dSPACE source FMU)
   Revision 2: interface/runtime audit fixes.
   Markers: [FIX n] = changed vs. the version you pasted.
   ============================================================ */

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Symbols become VIRYA_AEB_fmi2xxx. This define MUST come before the
   include. (The copy inside VIRYA_AEB.fmu defines it AFTER the include,
   which leaves the symbols unprefixed.)  [FIX 1]
   FMI2_ACTUAL_FUNCTION_PREFIX is set by fmi2Functions.h itself. */
#define FMI2_FUNCTION_PREFIX VIRYA_AEB_

#include "fmi2Functions.h"


/* ============================================================
   VIRYA AEB - Validated Parameters (UNCHANGED)
   ============================================================ */

#define TTC_WARNING   4.00
#define TTC_LEVEL_1   3.00
#define TTC_LEVEL_2   2.25
#define TTC_LEVEL_3   1.75

#define A_LEVEL_1    -2.0
#define A_LEVEL_2    -4.0
#define A_LEVEL_3    -6.0

#define MIN_BRAKE_DURATION 0.20

/* Default step if the importer ever passes an invalid one is NOT used:
   the real communicationStepSize is used (see fmi2DoStep).  [FIX 7] */
#define DEFAULT_STEP_SIZE  0.05   /* only for documentation / XML check */

/* Guards float accumulation when comparing state_time to 0.20 s. [FIX 8] */
#define TIME_EPS 1e-9

/* TTC reported when the ego vehicle is not closing on the target.
   Set to the value your Python reference uses. */
#define TTC_NO_COLLISION 999.0

/* [FIX 3] Valid-hex GUID. MUST be identical (including braces) to the
   guid="..." attribute in modelDescription.xml. */
#define FMU_GUID "{7f3c9b21-6a42-4e85-91d7-000000000001}"


/* ============================================================
   Value references - MUST MATCH modelDescription.xml
   ============================================================ */

#define VR_EGO_SPEED       1000u   /* Real, input  start 1.0 */
#define VR_TARGET_SPEED    1001u   /* Real, input  start 0.0 */
#define VR_DISTANCE        1002u   /* Real, input  start 5.0 */

#define VR_REL_VELOCITY    2000u   /* Real, output */
#define VR_TTC             2001u   /* Real, output */
#define VR_AEB_STATE       2002u   /* Integer, output */
#define VR_DESIRED_DECEL   2003u   /* Real, output */

#define VR_STATE_TIME      4000u   /* Real, local */

#define START_EGO_SPEED    1.0
#define START_TARGET_SPEED 0.0
#define START_DISTANCE     5.0


/* ============================================================
   AEB States (UNCHANGED)
   ============================================================ */

#define STATE_SAFE       0
#define STATE_WARNING    1
#define STATE_LEVEL_1    2
#define STATE_LEVEL_2    3
#define STATE_LEVEL_3    4


/* ============================================================
   FMU Instance
   ============================================================ */

typedef enum { LC_INSTANTIATED = 0, LC_INIT_MODE, LC_STEPPING, LC_TERMINATED } Lifecycle;

typedef struct
{
    /* [FIX 4] keep the callback needed to free the instance */
    fmi2CallbackFreeMemory freeMemory;
    Lifecycle lifecycle;

    int    current_state;
    double state_time;

    double ego_speed;
    double target_speed;
    double distance;

    double relative_velocity;
    double ttc;
    double desired_deceleration;

} VIRYA_AEB_Instance;


static void reset_instance(VIRYA_AEB_Instance* inst)
{
    inst->lifecycle = LC_INSTANTIATED;
    inst->current_state = STATE_SAFE;
    inst->state_time = 0.0;

    /* [FIX 5] defaults equal the start values in modelDescription.xml */
    inst->ego_speed    = START_EGO_SPEED;
    inst->target_speed = START_TARGET_SPEED;
    inst->distance     = START_DISTANCE;

    inst->relative_velocity = 0.0;
    inst->ttc = TTC_NO_COLLISION;
    inst->desired_deceleration = 0.0;
}


/* ============================================================
   FMI common
   ============================================================ */

/* [FIX 9] required by FMI 2.0, missing before */
FMI2_Export const char* fmi2GetTypesPlatform(void)
{
    return fmi2TypesPlatform;
}

FMI2_Export const char* fmi2GetVersion(void)
{
    return fmi2Version;
}

FMI2_Export fmi2Status fmi2SetDebugLogging(
    fmi2Component c,
    fmi2Boolean loggingOn,
    size_t nCategories,
    const fmi2String categories[])
{
    (void)loggingOn;
    (void)nCategories;
    (void)categories;

    return (c == NULL) ? fmi2Error : fmi2OK;
}


/* ============================================================
   Instantiate / Free
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
    VIRYA_AEB_Instance* inst;

    (void)instanceName;
    (void)fmuResourceLocation;
    (void)visible;
    (void)loggingOn;

    if (fmuType != fmi2CoSimulation)
        return NULL;

    /* [FIX 3] validate GUID against the one in modelDescription.xml */
    if (fmuGUID == NULL || strcmp(fmuGUID, FMU_GUID) != 0)
        return NULL;

    if (functions == NULL ||
        functions->allocateMemory == NULL ||
        functions->freeMemory == NULL)
        return NULL;

    inst = (VIRYA_AEB_Instance*)
        functions->allocateMemory(1, sizeof(VIRYA_AEB_Instance));

    if (inst == NULL)
        return NULL;

    reset_instance(inst);
    inst->freeMemory = functions->freeMemory;   /* [FIX 4] */

    return (fmi2Component)inst;
}

FMI2_Export void fmi2FreeInstance(fmi2Component c)
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;

    /* [FIX 4] FMI 2.0: the FMU frees what it allocated via the callback */
    if (inst != NULL && inst->freeMemory != NULL)
    {
        fmi2CallbackFreeMemory fm = inst->freeMemory;
        fm(inst);
    }
}


/* ============================================================
   Lifecycle
   ============================================================ */

FMI2_Export fmi2Status fmi2SetupExperiment(
    fmi2Component c,
    fmi2Boolean toleranceDefined,
    fmi2Real tolerance,
    fmi2Real startTime,
    fmi2Boolean stopTimeDefined,
    fmi2Real stopTime)
{
    (void)toleranceDefined;
    (void)tolerance;
    (void)startTime;      /* [FIX 6] model is time-invariant: any start time is fine */
    (void)stopTimeDefined;
    (void)stopTime;

    return (c == NULL) ? fmi2Error : fmi2OK;
}

FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;
    if (inst == NULL) return fmi2Error;

    inst->lifecycle = LC_INIT_MODE;
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;
    if (inst == NULL) return fmi2Error;

    /* Inputs set during initialization are kept. */
    inst->current_state = STATE_SAFE;
    inst->state_time = 0.0;
    inst->lifecycle = LC_STEPPING;
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;
    if (inst == NULL) return fmi2Error;

    inst->lifecycle = LC_TERMINATED;
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;
    fmi2CallbackFreeMemory fm;
    if (inst == NULL) return fmi2Error;

    fm = inst->freeMemory;
    reset_instance(inst);
    inst->freeMemory = fm;
    return fmi2OK;
}


/* ============================================================
   Get / Set
   ============================================================ */

FMI2_Export fmi2Status fmi2GetReal(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    fmi2Real value[])
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;
    size_t i;

    if (inst == NULL) return fmi2Error;
    if (nvr > 0 && (vr == NULL || value == NULL)) return fmi2Error;

    for (i = 0; i < nvr; i++)
    {
        switch (vr[i])
        {
            case VR_EGO_SPEED:     value[i] = inst->ego_speed;            break;
            case VR_TARGET_SPEED:  value[i] = inst->target_speed;         break;
            case VR_DISTANCE:      value[i] = inst->distance;             break;
            case VR_REL_VELOCITY:  value[i] = inst->relative_velocity;    break;
            case VR_TTC:           value[i] = inst->ttc;                  break;
            case VR_DESIRED_DECEL: value[i] = inst->desired_deceleration; break;
            /* [FIX 2] local variable: dSPACE reads it ({Discrete, Local}) */
            case VR_STATE_TIME:    value[i] = inst->state_time;           break;
            default:               return fmi2Error;   /* unknown reference */
        }
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2SetReal(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    const fmi2Real value[])
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;
    size_t i;

    if (inst == NULL || inst->lifecycle == LC_TERMINATED) return fmi2Error;
    if (nvr > 0 && (vr == NULL || value == NULL)) return fmi2Error;

    for (i = 0; i < nvr; i++)
    {
        switch (vr[i])
        {
            case VR_EGO_SPEED:    inst->ego_speed    = value[i]; break;
            case VR_TARGET_SPEED: inst->target_speed = value[i]; break;
            case VR_DISTANCE:     inst->distance     = value[i]; break;
            default:              return fmi2Error;  /* outputs/local/unknown */
        }
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2GetInteger(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    fmi2Integer value[])
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;
    size_t i;

    if (inst == NULL) return fmi2Error;
    if (nvr > 0 && (vr == NULL || value == NULL)) return fmi2Error;

    for (i = 0; i < nvr; i++)
    {
        if (vr[i] == VR_AEB_STATE)
            value[i] = (fmi2Integer)inst->current_state;
        else
            return fmi2Error;
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2SetInteger(
    fmi2Component c,
    const fmi2ValueReference vr[],
    size_t nvr,
    const fmi2Integer value[])
{
    (void)vr;
    (void)value;
    /* no integer inputs in this model */
    return (c == NULL || nvr > 0) ? fmi2Error : fmi2OK;
}

/* [FIX 9] remaining FMI 2.0 Co-Simulation functions, previously missing */
FMI2_Export fmi2Status fmi2GetBoolean(
    fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
    (void)vr; (void)value;
    return (c == NULL || nvr > 0) ? fmi2Error : fmi2OK;
}

FMI2_Export fmi2Status fmi2SetBoolean(
    fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
    (void)vr; (void)value;
    return (c == NULL || nvr > 0) ? fmi2Error : fmi2OK;
}

FMI2_Export fmi2Status fmi2GetString(
    fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
    (void)vr; (void)value;
    return (c == NULL || nvr > 0) ? fmi2Error : fmi2OK;
}

FMI2_Export fmi2Status fmi2SetString(
    fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
    (void)vr; (void)value;
    return (c == NULL || nvr > 0) ? fmi2Error : fmi2OK;
}

FMI2_Export fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
{ (void)c; (void)FMUstate; return fmi2Error; }

FMI2_Export fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate)
{ (void)c; (void)FMUstate; return fmi2Error; }

FMI2_Export fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
{ (void)c; (void)FMUstate; return fmi2Error; }

FMI2_Export fmi2Status fmi2SerializedFMUstateSize(
    fmi2Component c, fmi2FMUstate FMUstate, size_t* size)
{ (void)c; (void)FMUstate; (void)size; return fmi2Error; }

FMI2_Export fmi2Status fmi2SerializeFMUstate(
    fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size)
{ (void)c; (void)FMUstate; (void)serializedState; (void)size; return fmi2Error; }

FMI2_Export fmi2Status fmi2DeSerializeFMUstate(
    fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate* FMUstate)
{ (void)c; (void)serializedState; (void)size; (void)FMUstate; return fmi2Error; }

FMI2_Export fmi2Status fmi2GetDirectionalDerivative(
    fmi2Component c,
    const fmi2ValueReference vUnknown_ref[], size_t nUnknown,
    const fmi2ValueReference vKnown_ref[], size_t nKnown,
    const fmi2Real dvKnown[], fmi2Real dvUnknown[])
{
    (void)c; (void)vUnknown_ref; (void)nUnknown; (void)vKnown_ref;
    (void)nKnown; (void)dvKnown; (void)dvUnknown;
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2SetRealInputDerivatives(
    fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
    const fmi2Integer order[], const fmi2Real value[])
{
    (void)vr; (void)order; (void)value;
    return (c == NULL || nvr > 0) ? fmi2Error : fmi2OK;
}

FMI2_Export fmi2Status fmi2GetRealOutputDerivatives(
    fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
    const fmi2Integer order[], fmi2Real value[])
{
    (void)vr; (void)order; (void)value;
    return (c == NULL || nvr > 0) ? fmi2Error : fmi2OK;
}

FMI2_Export fmi2Status fmi2CancelStep(fmi2Component c)
{ (void)c; return fmi2Error; }   /* synchronous FMU: nothing to cancel */

FMI2_Export fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value)
{ (void)c; (void)s; (void)value; return fmi2Discard; }

FMI2_Export fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value)
{ (void)c; (void)s; (void)value; return fmi2Discard; }

FMI2_Export fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value)
{ (void)c; (void)s; (void)value; return fmi2Discard; }

FMI2_Export fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value)
{ (void)c; (void)s; (void)value; return fmi2Discard; }

FMI2_Export fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value)
{ (void)c; (void)s; (void)value; return fmi2Discard; }


/* ============================================================
   AEB FSM  (transitions UNCHANGED; only dt is now a parameter
   and the 0.20 s comparison has a tiny float tolerance)
   ============================================================ */

static int min_duration_reached(const VIRYA_AEB_Instance* instance)
{
    return (instance->state_time + TIME_EPS) >= MIN_BRAKE_DURATION;
}

static int update_aeb_state(
    VIRYA_AEB_Instance* instance,
    double ttc,
    double v_rel,
    double dt)
{
    int state = instance->current_state;

    instance->state_time += dt;

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
        else if (min_duration_reached(instance))
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
        else if (min_duration_reached(instance))
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
        if (min_duration_reached(instance))
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
   One AEB cycle per communication step:
   inputs -> v_rel, TTC -> FSM -> desired deceleration
   ============================================================ */

FMI2_Export fmi2Status fmi2DoStep(
    fmi2Component c,
    fmi2Real currentCommunicationPoint,
    fmi2Real communicationStepSize,
    fmi2Boolean noSetFMUStatePriorToCurrentPoint)
{
    VIRYA_AEB_Instance* inst = (VIRYA_AEB_Instance*)c;
    double v_rel;
    double ttc;
    int state;

    (void)currentCommunicationPoint;
    (void)noSetFMUStatePriorToCurrentPoint;

    if (inst == NULL || inst->lifecycle == LC_TERMINATED)
        return fmi2Error;

    /* [FIX 7] use the real communication step, reject nonsense */
    if (!(communicationStepSize > 0.0) || !isfinite(communicationStepSize))
        return fmi2Error;

    /* [FIX 10] invalid sensor values must not silently look "safe" */
    if (!isfinite(inst->ego_speed) || !isfinite(inst->target_speed) ||
        !isfinite(inst->distance))
        return fmi2Error;

    v_rel = inst->ego_speed - inst->target_speed;

    /* [FIX 11] closing + distance <= 0 means TTC = 0 (collision),
       NOT "no collision". Previously distance<=0 gave TTC=999 and SAFE. */
    if (v_rel > 0.0)
        ttc = ((inst->distance > 0.0) ? inst->distance : 0.0) / v_rel;
    else
        ttc = TTC_NO_COLLISION;

    state = update_aeb_state(inst, ttc, v_rel, communicationStepSize);

    inst->relative_velocity = v_rel;
    inst->ttc = ttc;

    switch (state)
    {
        case STATE_LEVEL_1: inst->desired_deceleration = A_LEVEL_1; break;
        case STATE_LEVEL_2: inst->desired_deceleration = A_LEVEL_2; break;
        case STATE_LEVEL_3: inst->desired_deceleration = A_LEVEL_3; break;
        default:            inst->desired_deceleration = 0.0;       break;
    }

    return fmi2OK;
}

/* ============================================================
   End of VIRYA AEB FMU
   ============================================================ */