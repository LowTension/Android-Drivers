#ifndef __GMFUSION
#define __GMFUSION

extern void Cross(float *a, float *b, float *c);

extern void MatrixTranspose(float *in, float*out);

extern void MatrixMultiply(float *in1, float*in2, float*out);

extern void RMtoRV(float *rm, float* rn);
extern void RMtoRVector(float* rm, float* rv);


extern void sfusion(float*in, float*out,float SampleRate);

extern void gMtoRM(float* g,float*M,float* RM);

#endif