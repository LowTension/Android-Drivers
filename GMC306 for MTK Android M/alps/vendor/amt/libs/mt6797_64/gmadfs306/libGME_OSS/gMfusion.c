#include "gMfusion.h"
#include "string.h"
#include "math.h"
float RM[9], RMn_1[9], RMn_1T[9], Xb[3], eta_n[4], Tmp[9];

void sfusion(float*in, float*out,float SampleRate)
{
	float *Xx, *Xy, *Xz;                           
	Xx = (float*)&RM[0];
	Xy = (float*)&RM[3];
	Xz = (float*)&RM[6];
	memcpy(Xz, &in[0], 3 * sizeof(float));
	memcpy(Xb, &in[3], 3 * sizeof(float));
	Cross(Xb, Xz, Xx);		
	Cross(Xz, Xx, Xy);	
	MatrixTranspose(RM, Tmp); memcpy(RM, Tmp, sizeof(float)* 9);
	MatrixTranspose(RMn_1, Tmp); memcpy(RMn_1T, Tmp, sizeof(float)* 9);
	MatrixMultiply(RM, RMn_1T, Tmp);
	RMtoRV(Tmp, eta_n);
	memcpy(RMn_1, RM, sizeof(float)* 9);
	memcpy(out, RM, sizeof(float)* 6);
	out[6]=eta_n[0]*eta_n[3]*SampleRate;     //rad/sec
	out[7]=eta_n[1]*eta_n[3]*SampleRate;     //rad/sec
	out[8]=eta_n[2]*eta_n[3]*SampleRate;     //rad/sec
}

void Cross(float *a, float *b, float *c)
{ 
        float len;
        len=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
        a[0]/=len;a[1]/=len;a[2]/=len;
        len=sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
        b[0]/=len;b[1]/=len;b[2]/=len;        
#define x 0
#define y 1
#define z 2
	c[x] = a[y] * b[z] - a[z] * b[y];
	c[y] = a[z] * b[x] - a[x] * b[z];
	c[z] = a[x] * b[y] - a[y] * b[x];	
				
}

void MatrixTranspose(float *in, float*out)
{
	out[0] = in[0];
	out[1] = in[3];
	out[2] = in[6];
	out[3] = in[1];
	out[4] = in[4];
	out[5] = in[7];
	out[6] = in[2];
	out[7] = in[5];
	out[8] = in[8];
}

void MatrixMultiply(float *in1, float*in2, float*out)
{
	int i, j, k;
	memset(out, 0, 9 * sizeof(float));
	for (i = 0; i<3; i++)
	{
		for (j = 0; j<3; j++)
		{
			for (k = 0; k<3; k++)
			{
				out[i * 3 + j] += in1[i * 3 + k] * in2[j + 3 * k];
			}
		}
	}
}

void RMtoRV(float* rm, float* rv)
{             
	static float srv[4];
        rv[0]=0;rv[1]=0;rv[2]=0;rv[3]=0;
        float tr=rm[0]+rm[4]+rm[8];
        float sin_eta=0.0;
        if (tr>3 )
            tr=3;
        if (tr<-1)
            tr=-1;
        rv[3]=acos((tr-1)/2);//arc
        if (rv[3]*180.0/acos(-1.0)<0.5)
        {         
            rv[0]=srv[0];	rv[1]=srv[1];	rv[2]=srv[2];	rv[3]=srv[3];           //0
        }
        else if(rv[3]*180.0/acos(-1.0)>179.5)
        {     
            rv[0]=srv[0];	rv[1]=srv[1];	rv[2]=srv[2];	rv[3]=srv[3];           //0
        }
		else
        {
            sin_eta=2*sin(rv[3]);
            rv[0]=(rm[5]-rm[7])/sin_eta;
            rv[1]=(rm[6]-rm[2])/sin_eta;
            rv[2]=(rm[1]-rm[3])/sin_eta;  
            srv[0]=rv[0];	srv[1]=rv[1];	srv[2]=rv[2];	srv[3]=rv[3];           //0
        }
    

}

void RMtoRVector(float* rm, float* rv)
{             
	static float srv[4];
        rv[0]=0;rv[1]=0;rv[2]=0;rv[3]=0;
        float tr=rm[0]+rm[4]+rm[8];
        float sin_eta=0.0;
        if (tr>3 )
            tr=3;
        if (tr<-1)
            tr=-1;
        rv[3]=acos((tr-1)/2);//arc
        if (rv[3]*180.0/acos(-1.0)<0.5)
        {         
            rv[0]=srv[0];	rv[1]=srv[1];	rv[2]=srv[2];	rv[3]=srv[3];           //0
        }
        else if(rv[3]*180.0/acos(-1.0)>179.5)
        {     
            rv[0]=srv[0];	rv[1]=srv[1];	rv[2]=srv[2];	rv[3]=srv[3];           //0
        }
	else
        {
            sin_eta=2*sin(rv[3]);
            rv[0]=(rm[5]-rm[7])/sin_eta;
            rv[1]=(rm[6]-rm[2])/sin_eta;
            rv[2]=(rm[1]-rm[3])/sin_eta;  
            srv[0]=rv[0];	srv[1]=rv[1];	srv[2]=rv[2];	srv[3]=rv[3];           //0
        }
      

}
void gMtoRM(float* g,float* M,float *m)
{
	float a[3],b[3],c[3];
	a[0]=g[0];a[1]=g[1];a[2]=g[2];//graviry
        b[0]=M[0];b[1]=M[1];b[2]=M[2];//mag

        Cross(b,a,c);//c=Axis X
        Cross(a,c,b);//b=Axis Y

        m[0]=c[0];m[1]=b[0];m[2]=a[0];
        m[3]=c[1];m[4]=b[1];m[5]=a[1];
        m[6]=c[2];m[7]=b[2];m[8]=a[2];	      
}
    


