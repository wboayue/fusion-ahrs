// Pointer-based wrappers around the Fusion C API. FusionVector and
// FusionQuaternion are unions, so they are not passed by value across FFI.

#include "Fusion.h"
#include <stdlib.h>

typedef struct {
    float quaternion[4]; // w, x, y, z
    float gravity[3];
    float linearAcceleration[3];
    float earthAcceleration[3];
    FusionAhrsInternalStates internalStates;
    FusionAhrsFlags flags;
} ShimOutputs;

static FusionVector Vector(const float v[3]) {
    const FusionVector result = {.axis = {.x = v[0], .y = v[1], .z = v[2]}};
    return result;
}

static void Copy(const FusionVector v, float out[3]) {
    out[0] = v.axis.x;
    out[1] = v.axis.y;
    out[2] = v.axis.z;
}

FusionAhrs *ShimAhrsNew(const FusionAhrsSettings *const settings) {
    FusionAhrs *const ahrs = malloc(sizeof(FusionAhrs));
    FusionAhrsInitialise(ahrs);
    FusionAhrsSetSettings(ahrs, settings);
    return ahrs;
}

void ShimAhrsFree(FusionAhrs *const ahrs) {
    free(ahrs);
}

void ShimAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings) {
    FusionAhrsSetSettings(ahrs, settings);
}

void ShimAhrsSetSamplePeriod(FusionAhrs *const ahrs, const float samplePeriod) {
    FusionAhrsSetSamplePeriod(ahrs, samplePeriod);
}

void ShimAhrsRestart(FusionAhrs *const ahrs) {
    FusionAhrsRestart(ahrs);
}

void ShimAhrsSkipStartup(FusionAhrs *const ahrs) {
    FusionAhrsSkipStartup(ahrs);
}

void ShimAhrsUpdate(FusionAhrs *const ahrs, const float gyroscope[3], const float accelerometer[3], const float magnetometer[3]) {
    FusionAhrsUpdate(ahrs, Vector(gyroscope), Vector(accelerometer), Vector(magnetometer));
}

void ShimAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const float gyroscope[3], const float accelerometer[3]) {
    FusionAhrsUpdateNoMagnetometer(ahrs, Vector(gyroscope), Vector(accelerometer));
}

void ShimAhrsUpdateExternalHeading(FusionAhrs *const ahrs, const float gyroscope[3], const float accelerometer[3], const float heading) {
    FusionAhrsUpdateExternalHeading(ahrs, Vector(gyroscope), Vector(accelerometer), heading);
}

void ShimAhrsSetQuaternion(FusionAhrs *const ahrs, const float quaternion[4]) {
    const FusionQuaternion q = {.element = {.w = quaternion[0], .x = quaternion[1], .y = quaternion[2], .z = quaternion[3]}};
    FusionAhrsSetQuaternion(ahrs, q);
}

void ShimAhrsSetHeading(FusionAhrs *const ahrs, const float heading) {
    FusionAhrsSetHeading(ahrs, heading);
}

void ShimAhrsOutputs(const FusionAhrs *const ahrs, ShimOutputs *const outputs) {
    const FusionQuaternion q = FusionAhrsGetQuaternion(ahrs);
    outputs->quaternion[0] = q.element.w;
    outputs->quaternion[1] = q.element.x;
    outputs->quaternion[2] = q.element.y;
    outputs->quaternion[3] = q.element.z;
    Copy(FusionAhrsGetGravity(ahrs), outputs->gravity);
    Copy(FusionAhrsGetLinearAcceleration(ahrs), outputs->linearAcceleration);
    Copy(FusionAhrsGetEarthAcceleration(ahrs), outputs->earthAcceleration);
    outputs->internalStates = FusionAhrsGetInternalStates(ahrs);
    outputs->flags = FusionAhrsGetFlags(ahrs);
}

FusionBias *ShimBiasNew(const FusionBiasSettings *const settings) {
    FusionBias *const bias = malloc(sizeof(FusionBias));
    FusionBiasInitialise(bias);
    FusionBiasSetSettings(bias, settings);
    return bias;
}

void ShimBiasFree(FusionBias *const bias) {
    free(bias);
}

void ShimBiasUpdate(FusionBias *const bias, const float gyroscope[3], float out[3]) {
    Copy(FusionBiasUpdate(bias, Vector(gyroscope)), out);
}

void ShimBiasGetOffset(const FusionBias *const bias, float out[3]) {
    Copy(FusionBiasGetOffset(bias), out);
}

float ShimCompass(const float accelerometer[3], const float magnetometer[3], const FusionConvention convention) {
    return FusionCompass(Vector(accelerometer), Vector(magnetometer), convention);
}

void ShimRemap(const float sensor[3], const FusionRemapAlignment alignment, float out[3]) {
    Copy(FusionRemap(Vector(sensor), alignment), out);
}

const char *ShimConventionToString(const FusionConvention convention) {
    return FusionConventionToString(convention);
}

const char *ShimRemapAlignmentToString(const FusionRemapAlignment alignment) {
    return FusionRemapAlignmentToString(alignment);
}

void ShimModelInertial(const float uncalibrated[3], const float misalignment[9], const float sensitivity[3], const float offset[3], float out[3]) {
    FusionMatrix m; // row-major
    for (int i = 0; i < 9; i++) {
        m.array[i] = misalignment[i];
    }
    Copy(FusionModelInertial(Vector(uncalibrated), m, Vector(sensitivity), Vector(offset)), out);
}

void ShimModelMagnetic(const float uncalibrated[3], const float softIronMatrix[9], const float hardIronOffset[3], float out[3]) {
    FusionMatrix m; // row-major
    for (int i = 0; i < 9; i++) {
        m.array[i] = softIronMatrix[i];
    }
    Copy(FusionModelMagnetic(Vector(uncalibrated), m, Vector(hardIronOffset)), out);
}

//------------------------------------------------------------------------------
// FusionMath.h wrappers

static FusionQuaternion Quaternion(const float q[4]) {
    const FusionQuaternion result = {.element = {.w = q[0], .x = q[1], .y = q[2], .z = q[3]}};
    return result;
}

static void CopyQuaternion(const FusionQuaternion q, float out[4]) {
    out[0] = q.element.w;
    out[1] = q.element.x;
    out[2] = q.element.y;
    out[3] = q.element.z;
}

static FusionMatrix Matrix(const float m[9]) {
    FusionMatrix result; // row-major
    for (int i = 0; i < 9; i++) {
        result.array[i] = m[i];
    }
    return result;
}

static void CopyMatrix(const FusionMatrix m, float out[9]) {
    for (int i = 0; i < 9; i++) {
        out[i] = m.array[i];
    }
}

float ShimDegreesToRadians(const float degrees) { return FusionDegreesToRadians(degrees); }
float ShimRadiansToDegrees(const float radians) { return FusionRadiansToDegrees(radians); }
float ShimArcSin(const float value) { return FusionArcSin(value); }

bool ShimVectorIsZero(const float v[3]) { return FusionVectorIsZero(Vector(v)); }
void ShimVectorAdd(const float a[3], const float b[3], float out[3]) { Copy(FusionVectorAdd(Vector(a), Vector(b)), out); }
void ShimVectorSubtract(const float a[3], const float b[3], float out[3]) { Copy(FusionVectorSubtract(Vector(a), Vector(b)), out); }
void ShimVectorScale(const float v[3], const float s, float out[3]) { Copy(FusionVectorScale(Vector(v), s), out); }
float ShimVectorSum(const float v[3]) { return FusionVectorSum(Vector(v)); }
void ShimVectorHadamard(const float a[3], const float b[3], float out[3]) { Copy(FusionVectorHadamard(Vector(a), Vector(b)), out); }
void ShimVectorCross(const float a[3], const float b[3], float out[3]) { Copy(FusionVectorCross(Vector(a), Vector(b)), out); }
float ShimVectorDot(const float a[3], const float b[3]) { return FusionVectorDot(Vector(a), Vector(b)); }
float ShimVectorNormSquared(const float v[3]) { return FusionVectorNormSquared(Vector(v)); }
float ShimVectorNorm(const float v[3]) { return FusionVectorNorm(Vector(v)); }
void ShimVectorNormalise(const float v[3], float out[3]) { Copy(FusionVectorNormalise(Vector(v)), out); }

void ShimQuaternionAdd(const float a[4], const float b[4], float out[4]) { CopyQuaternion(FusionQuaternionAdd(Quaternion(a), Quaternion(b)), out); }
void ShimQuaternionScale(const float q[4], const float s, float out[4]) { CopyQuaternion(FusionQuaternionScale(Quaternion(q), s), out); }
float ShimQuaternionSum(const float q[4]) { return FusionQuaternionSum(Quaternion(q)); }
void ShimQuaternionHadamard(const float a[4], const float b[4], float out[4]) { CopyQuaternion(FusionQuaternionHadamard(Quaternion(a), Quaternion(b)), out); }
void ShimQuaternionProduct(const float a[4], const float b[4], float out[4]) { CopyQuaternion(FusionQuaternionProduct(Quaternion(a), Quaternion(b)), out); }
void ShimQuaternionVectorProduct(const float q[4], const float v[3], float out[4]) { CopyQuaternion(FusionQuaternionVectorProduct(Quaternion(q), Vector(v)), out); }
float ShimQuaternionNormSquared(const float q[4]) { return FusionQuaternionNormSquared(Quaternion(q)); }
float ShimQuaternionNorm(const float q[4]) { return FusionQuaternionNorm(Quaternion(q)); }
void ShimQuaternionNormalise(const float q[4], float out[4]) { CopyQuaternion(FusionQuaternionNormalise(Quaternion(q)), out); }
void ShimQuaternionToMatrix(const float q[4], float out[9]) { CopyMatrix(FusionQuaternionToMatrix(Quaternion(q)), out); }

void ShimQuaternionToEuler(const float q[4], float out[3]) {
    const FusionEuler euler = FusionQuaternionToEuler(Quaternion(q));
    out[0] = euler.angle.roll;
    out[1] = euler.angle.pitch;
    out[2] = euler.angle.yaw;
}

void ShimMatrixScale(const float m[9], const float s, float out[9]) { CopyMatrix(FusionMatrixScale(Matrix(m), s), out); }
void ShimMatrixMultiply(const float m[9], const float v[3], float out[3]) { Copy(FusionMatrixMultiply(Matrix(m), Vector(v)), out); }
