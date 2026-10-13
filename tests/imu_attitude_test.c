#include "svc/imu_attitude.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float a, float b, float eps) { assert(fabsf(a-b) < eps); }
static float yaw(const float q[4])
{ return atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])); }
int main(void)
{
    imu_attitude_t s;
    float q[4], raw[6]={0,0,1,0,0,0};
    const float pi=3.14159265358979323846f;
    imu_attitude_reset(&s);
    assert(imu_attitude_step(&s, raw, true, 0, q));
    near(q[0],1,1e-6f);
    for (int axis=0; axis<3; ++axis) {
        imu_attitude_reset(&s);
        raw[0]=raw[1]=0; raw[2]=1; raw[3]=raw[4]=raw[5]=0;
        assert(imu_attitude_step(&s,raw,true,0,q));
        raw[2]=2; /* Disable gravity correction to isolate gyro integration. */
        raw[3+axis]=90;
        int64_t now=0;
        for (int i=0; now<1000000; ++i) {
            int64_t dt=(i%2) ? 3000 : 1000;
            if (now+dt>1000000) dt=1000000-now;
            now+=dt;
            assert(imu_attitude_step(&s,raw,true,now,q));
        }
        near(q[0],sqrtf(0.5f),1e-4f); near(q[1+axis],sqrtf(0.5f),1e-4f);
        float norm=0; for(int i=0;i<4;++i) norm+=q[i]*q[i];
        near(norm,1,1e-5f);
    }
    /* Seed a known roll/pitch (30/30 degrees), no startup ramp from zero. */
    imu_attitude_reset(&s);
    raw[0]=-0.5f; raw[1]=sqrtf(3)/4; raw[2]=0.75f;
    raw[3]=raw[4]=raw[5]=0;
    assert(imu_attitude_step(&s,raw,true,0,q));
    near(asinf(2*(q[0]*q[2]-q[3]*q[1])),pi/6,1e-5f);
    near(atan2f(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2])),pi/6,1e-5f);
    /* Gravity restores roll/pitch during stationary sampling. */
    raw[0]=raw[1]=0; raw[2]=1;
    for (int i=1;i<=6000;++i) assert(imu_attitude_step(&s,raw,true,i*1000LL,q));
    near(q[1],0,1e-5f); near(q[2],0,1e-5f);
    /* Yaw integrates independently from gravity, is not forced to north. */
    imu_attitude_reset(&s); assert(imu_attitude_step(&s,raw,true,0,q));
    raw[5]=90;
    for(int i=1;i<=1000;++i) assert(imu_attitude_step(&s,raw,true,i*1000LL,q));
    near(yaw(q),pi/2,1e-4f);
    float before=yaw(q);
    assert(imu_attitude_step(&s,raw,true,1000100,q)); near(yaw(q),before,1e-6f);
    assert(!imu_attitude_step(&s,raw,false,1001000,q)); near(q[0],1,1e-6f);
    assert(imu_attitude_step(&s,raw,true,2000000,q)); near(yaw(q),before,1e-5f);
    assert(imu_attitude_step(&s,raw,true,3000000,q)); near(yaw(q),before,1e-5f);
    raw[3]=NAN; assert(!imu_attitude_step(&s,raw,true,3001000,q));
    raw[3]=0; raw[5]=0;
    assert(imu_attitude_step(&s,raw,true,3002000,q));
    assert(imu_attitude_step(&s,raw,true,0,q)); near(yaw(q),0,1e-5f);
    imu_attitude_reset(&s); raw[2]=0;
    assert(!imu_attitude_step(&s,raw,true,0,q));
    raw[2]=-1; assert(imu_attitude_step(&s,raw,true,1000,q));
    near(fabsf(q[1]),1,1e-5f); /* Upside down remains a valid pose. */
    puts("PASS: board quaternion axes, actual dt, normalization, tilt correction, yaw, gaps and invalid input");
    return 0;
}
