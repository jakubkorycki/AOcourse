#ifndef ZERNIKE_H
#define ZERNIKE_H


double factorial(int n);
void zernike_malacara_index_to_nm(int malacara, int *n, int *m);
int zernike_nm_to_malacara_index(int n, int m);
int *zernike_R_factors(int n, int m, int *factor);
int **prepare_zernike_R_factors(int max_order);
void dispose_zernike_R_factors(int **R_factors, int max_order);
double zernike_prepared(int n, int m, double rho, double phi, int *Rc);
double zernike(int n, int m, double rho, double phi);
double dzernike(int n, int m, double rho, double phi, double deltax, double deltay);


#endif // ZERNIKE_H
