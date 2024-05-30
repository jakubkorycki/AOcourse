#include <cmath>
#include <cstdlib>
#include "zernike.h"
#include "util.h"

#define Z_ALLOC_INC 20
int **z_stored=0;
int z_stored_max_order=-1;


// --- Zernike polynomials routines below -------------------------------------------------------------------------
// short and staigthforward
double factorial(int n)
{
    double result=1;
    while(n>0) { result*=n--; }
    return result;
}

// helper function for one-index ordering of Zernike polynomyals
// malacara indexing starts from 1 (for piston n=m=0)
void zernike_malacara_index_to_nm(int malacara, int *n, int *m)
{
    *n=ceil((sqrt(1.0+8*malacara)-1)/2-1);
    int n0=(*n)*(*n+1)/2;
    int m0=(malacara-n0);
    *m=*n-2*(m0-1);
}

// helper function for one-index ordering of Zernike polynomyals
// malacara indexing starts from 1 (for piston, n=m=0)
int zernike_nm_to_malacara_index(int n, int m)
{
    if(m>n || (n+m)%2) return -1; // invalid indices
    return (n-m)/2+n*(n+1)/2+1;
}


// precalculate factors of radial Zernike polynomial.
// the coefficients are stored in factor array (dynamically allocated)
// and returned by the function. Can be used e.g. by zernike_prepared function.
int *zernike_R_factors(int n, int m, int *factor)
{
    int i,s;
    if(!factor) factor=(int*)malloc((n+1)*sizeof(int));
    for(i=0; i<=n; i++) factor[i]=0;
    m=abs(m);
    for(s=0; s<=(n-m)/2; s++)
        *(factor+n-2*s)=(s&1?-1:1)*factorial(n-s)/(factorial(s)*factorial((n+m)/2-s)*factorial((n-m)/2-s));
    return factor;
}

// pre-calculate factors (radial) for whole bunch of polynomials, up to max_order
int **prepare_zernike_R_factors(int max_order)
{
    int **R_factors=(int**)malloc(max_order*sizeof(int*));
    for(int i=0; i<max_order; i++)
    {
        int n,m;
        int *factor=(int *)malloc(max_order*sizeof(int*));
        for(int j=0; j<max_order; j++) factor[j]=0;
        zernike_malacara_index_to_nm(i,&n,&m);
        R_factors[i]=zernike_R_factors(n,m,factor);
    }
    return R_factors;
}

void dispose_zernike_R_factors(int **R_factors, int max_order)
{
    for(int i=0; i<max_order; i++) free(R_factors[i]);
    free(R_factors);
}



// calculate a value of Zernike polynomial Z_n^m at given point (rho, phi).
// uses array of precalculated factors Rc for better performance.
double zernike_prepared(int n, int m, double rho, double phi, int *Rc)
{
    double result=Rc[n];
    for(int i=n-1; i>=0; i--) result=result*rho+Rc[i];
    // it is used to be limited, but numeric calculating of derivative might require points outside of unit circle
    // if(rho>1) result=0;
    return (result*(m>=0?cos(m*phi):sin(-m*phi)));
    //return (result*(m>=0?sin(m*phi):cos(-m*phi))); // sin<->cos is opposite to standard definition, but in accord to traditional FS
}

//
double zernike(int n, int m, double rho, double phi)
{
    int idx=zernike_nm_to_malacara_index(n,m);
    //printf("%d %d ->%d; ",n,m,idx);
    if(idx<0) return 0; // throw runtime_error("Invalid indeces for Zernike polynomials");
    if(idx>=z_stored_max_order)
    {
        // calculate factors for greater maximum order
        if(z_stored) dispose_zernike_R_factors(z_stored, z_stored_max_order);
        //if(z_stored_max_order<=0) z_stored_max_order=0;
        z_stored_max_order=(int(idx/Z_ALLOC_INC)+1)*Z_ALLOC_INC;
        //printf("**new z_stored_max_order=%d\n",z_stored_max_order);
        z_stored=prepare_zernike_R_factors(z_stored_max_order);
    }
    //double norm=m!=0?sqrt(2*(n+1)):sqrt(n+1);
    return zernike_prepared(n,m,rho,phi,z_stored[idx]);//*norm;
}

// **FEXME: use analytical expression instead!
// Zernike derivative, numerically calculated.
// Either deltax or deltay increment applied (normally not both)
double dzernike(int n, int m, double rho, double phi, double deltax, double deltay)
{
    double x=rho*cos(phi);
    double y=rho*sin(phi);
    double rho1=sqrt(SQR(x-deltax)+SQR(y-deltay));
    double rho2=sqrt(SQR(x+deltax)+SQR(y+deltay));
    double phi1=atan2(y-deltay,x-deltax);
    double phi2=atan2(y+deltay,x+deltax);
    return (zernike(n,m,rho2,phi2)-zernike(n,m,rho1,phi1))/(2*sqrt(SQR(deltax)+SQR(deltay)));
}


