#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <ros/package.h>

#define Circle_Center_X  440 // 440 // 435
#define Circle_Center_Y  440 // 440 // 435
#define Mystep             100
#define Circle_inside_r  175
#define Circle_outside_r 440 // 440 // 435
#define region_max       900 // 100 times outside the outer ring
#define Rectangle_Probe
//#define Print_Detail
#define Print_Rectangle

#define X_PROBE_OFFSET 28    // 28.243 // X offset: -left  +right  [of the nozzle]
#define Y_PROBE_OFFSET -7    // -7.45  // Y offset: -front +behind [the nozzle]
#define Z_PROBE_OFFSET 11.75 // 12.7   // Z offset: -below +above  [the nozzle]

//*
#if defined(Rectangle_Probe)
double XYZ_Position[100][3] = {
    {0.00000, 0.00000, 0},     {100.00000, 0.00000, 0},   {200.00000, 71.21822, 0},   {300.00000, 22.86693, 0},  {400.00000, 1.82195, 0},
    {500.00000, 4.11011, 0},   {600.00000, 30.12197, 0},  {700.00000, 85.03521, 0},   {800.00000, 0.00000, 0},   {880.00000, 0.00000, 0},
    {0.00000, 100.00000, 0},   {100.00000, 160.71520, 0}, {200.00000, 100.00000, 0},  {300.00000, 100.00000, 0}, {400.00000, 100.00000, 0},
    {500.00000, 100.00000, 0}, {600.00000, 100.00000, 0}, {700.00000, 100.00000, 0},  {800.00000, 187.01779, 0}, {880.00000, 100.00000, 0},
    {71.21822, 200.00000, 0},  {100.00000, 200.00000, 0}, {200.00000, 200.00000, 0},  {300.00000, 200.00000, 0}, {400.00000, 200.00000, 0},
    {500.00000, 200.00000, 0}, {600.00000, 200.00000, 0}, {700.00000, 200.00000, 0},  {800.00000, 200.00000, 0}, {808.78178, 200.00000, 0},
    {22.86693, 300.00000, 0},  {100.00000, 300.00000, 0}, {200.00000, 300.00000, 0},  {300.00000, 300.00000, 0}, {400.00000, 269.63275, 0},
    {500.00000, 275.60718, 0}, {600.00000, 300.00000, 0}, {700.00000, 300.00000, 0},  {800.00000, 300.00000, 0}, {857.13307, 300.00000, 0},
    {1.82195, 400.00000, 0},   {100.00000, 400.00000, 0}, {200.00000, 400.00000, 0},  {300.00000, 335.00000, 0}, {400.00000, 400.00000, 0},
    {500.00000, 400.00000, 0}, {600.00000, 369.11277, 0}, {700.00000, 400.00000, 0},  {800.00000, 400.00000, 0}, {878.17805, 400.00000, 0},
    {4.11011, 500.00000, 0},   {100.00000, 500.00000, 0}, {200.00000, 500.00000, 0},  {300.00000, 545.00000, 0}, {400.00000, 500.00000, 0},
    {500.00000, 500.00000, 0}, {600.00000, 510.88723, 0}, {700.00000, 500.00000, 0},  {800.00000, 500.00000, 0}, {875.88989, 500.00000, 0},
    {30.12197, 600.00000, 0},  {100.00000, 600.00000, 0}, {209.837014, 615.00000, 0}, {300.00000, 615.00000, 0}, {400.00000, 615.00000, 0},
    {500.00000, 615.00000, 0}, {600.00000, 615.00000, 0}, {670.16299, 615.00000, 0},  {800.00000, 600.00000, 0}, {849.87803, 600.00000, 0},
    {85.03521, 700.00000, 0},  {100.00000, 700.00000, 0}, {209.837014, 715.00000, 0}, {300.00000, 715.00000, 0}, {400.00000, 715.00000, 0},
    {500.00000, 715.00000, 0}, {600.00000, 715.00000, 0}, {670.16299, 715.00000, 0},  {800.00000, 692.98221, 0}, {880.00000, 700.00000, 0},
    {0.00000, 800.00000, 0},   {100.00000, 719.28480, 0}, {209.837014, 815.00000, 0}, {300.00000, 815.00000, 0}, {400.00000, 815.00000, 0},
    {500.00000, 815.00000, 0}, {600.00000, 815.00000, 0}, {670.16299, 815.00000, 0},  {800.00000, 800.00000, 0}, {880.00000, 800.00000, 0},
    {0.00000, 880.00000, 0},   {100.00000, 880.00000, 0}, {200.00000, 808.78178, 0},  {300.00000, 857.13307, 0}, {400.00000, 878.17805, 0},
    {500.00000, 875.88989, 0}, {600.00000, 849.87803, 0}, {700.00000, 794.96479, 0},  {800.00000, 880.00000, 0}, {880.00000, 880.00000, 0}};

#else
double XYZ_Position[100][3] = {
    {0.00000, 0.00000, 0},     {100.00000, 0.00000, 0},   {200.00000, 71.21822, 0},  {300.00000, 22.86693, 0},  {400.00000, 1.82195, 0},
    {500.00000, 4.11011, 0},   {600.00000, 30.12197, 0},  {700.00000, 85.03521, 0},  {800.00000, 0.00000, 0},   {880.00000, 0.00000, 0},
    {0.00000, 100.00000, 0},   {100.00000, 160.71520, 0}, {200.00000, 100.00000, 0}, {300.00000, 100.00000, 0}, {400.00000, 100.00000, 0},
    {500.00000, 100.00000, 0}, {600.00000, 100.00000, 0}, {700.00000, 100.00000, 0}, {800.00000, 187.01779, 0}, {880.00000, 100.00000, 0},
    {71.21822, 200.00000, 0},  {100.00000, 200.00000, 0}, {200.00000, 200.00000, 0}, {300.00000, 200.00000, 0}, {400.00000, 200.00000, 0},
    {500.00000, 200.00000, 0}, {600.00000, 200.00000, 0}, {700.00000, 200.00000, 0}, {800.00000, 200.00000, 0}, {808.78178, 200.00000, 0},
    {22.86693, 300.00000, 0},  {100.00000, 300.00000, 0}, {200.00000, 300.00000, 0}, {300.00000, 300.00000, 0}, {400.00000, 269.63275, 0},
    {500.00000, 275.60718, 0}, {600.00000, 300.00000, 0}, {700.00000, 300.00000, 0}, {800.00000, 300.00000, 0}, {857.13307, 300.00000, 0},
    {1.82195, 400.00000, 0},   {100.00000, 400.00000, 0}, {200.00000, 400.00000, 0}, {300.00000, 335.00000, 0}, {400.00000, 400.00000, 0},
    {500.00000, 400.00000, 0}, {600.00000, 369.11277, 0}, {700.00000, 400.00000, 0}, {800.00000, 400.00000, 0}, {878.17805, 400.00000, 0},
    {4.11011, 500.00000, 0},   {100.00000, 500.00000, 0}, {200.00000, 500.00000, 0}, {300.00000, 545.00000, 0}, {400.00000, 500.00000, 0},
    {500.00000, 500.00000, 0}, {600.00000, 510.88723, 0}, {700.00000, 500.00000, 0}, {800.00000, 500.00000, 0}, {875.88989, 500.00000, 0},
    {30.12197, 600.00000, 0},  {100.00000, 600.00000, 0}, {200.00000, 600.00000, 0}, {300.00000, 600.00000, 0}, {400.00000, 610.36725, 0},
    {500.00000, 604.39282, 0}, {600.00000, 600.00000, 0}, {700.00000, 600.00000, 0}, {800.00000, 600.00000, 0}, {849.87803, 600.00000, 0},
    {85.03521, 700.00000, 0},  {100.00000, 700.00000, 0}, {200.00000, 700.00000, 0}, {300.00000, 700.00000, 0}, {400.00000, 700.00000, 0},
    {500.00000, 700.00000, 0}, {600.00000, 700.00000, 0}, {700.00000, 700.00000, 0}, {800.00000, 692.98221, 0}, {880.00000, 700.00000, 0},
    {0.00000, 800.00000, 0},   {100.00000, 719.28480, 0}, {200.00000, 800.00000, 0}, {300.00000, 800.00000, 0}, {400.00000, 800.00000, 0},
    {500.00000, 800.00000, 0}, {600.00000, 800.00000, 0}, {700.00000, 794.96479, 0}, {800.00000, 800.00000, 0}, {880.00000, 800.00000, 0},
    {0.00000, 880.00000, 0},   {100.00000, 880.00000, 0}, {200.00000, 808.78178, 0}, {300.00000, 857.13307, 0}, {400.00000, 878.17805, 0},
    {500.00000, 875.88989, 0}, {600.00000, 849.87803, 0}, {700.00000, 794.96479, 0}, {800.00000, 880.00000, 0}, {880.00000, 880.00000, 0}};

#endif
//*/

//
double XYZ_Position1_Add[13][3] = {{160.71520, 100.00000, 0}, {719.28480, 100.00000, 0}, {335.00000, 300.00000, 0}, {545.00000, 300.00000, 0},
                                   {269.63275, 400.00000, 0}, {610.36725, 400.00000, 0}, {275.60718, 500.00000, 0}, {604.39282, 500.00000, 0},
                                   {369.11277, 600.00000, 0}, {510.88723, 600.00000, 0}, {794.96479, 700.00000, 0}, {187.01779, 800.00000, 0},
                                   {692.98221, 800.00000, 0}};

//*/

double differ_ab_1[100][3] = {0}, differ_ac_1[100][3] = {0};
double differ_ab_2[100][3] = {0}, differ_ac_2[100][3] = {0};
double a_1[100], b_1[100], c_1[100], d_1[100];
double a_2[100], b_2[100], c_2[100], d_2[100];

double Ja[100][5] = {0}, Jb[100][5] = {0}, Jc[100][5] = {0};
double Ja_Add[100][5], Jb_Add[100][5], Jc_Add[100][5];

double x_pos, y_pos, z_pos;
double r_in, r_out;
double sol1_i, sol2_i, sol1_o, sol2_o;
static double error[2] = {-999, -999};
double *temp, *temp1, *temp2, *temp3;

double temp_calc_x, temp_calc_y, temp_calc_z[3];
double temp_calc_Joint[3][5];

int total;

bool circle_inside(double, double);
bool circle_outside(double, double);
bool inside_circle_in(double, double);
bool outer_circle_in(double, double);
bool In_Rectangle(double, double);
void outer_circle_pos(double);
void inside_circle_pos(double);
void outer_circle_pos1(double);
void print_012(void);
void print_point(void);
double *inside_circle_posx_d(double);
double *inside_circle_posy_d(double);
double *outer_circle_posx_d(double);
double *outer_circle_posy_d(double);

void circle_print(void);
void for_find_in_out_point(void);
void for_use_Y_find_X(void);
void for_use_X_find_Y(void);
void clear_data(void);
bool find_region_in_out(double, double);
bool find_region_in_out_d(double, double);

void Produce_Mesh(void);
void Produce_Joint_XYZ(void);
void Load_Mesh(double);
void Calc_abcd(void);
double calc_z(double, double);
double calc_z_rectangle(double, double, bool);
void XYZ_transformation_Joint_abc(void);

bool circle_inside(double cir_x, double cir_y)
{
    if (pow((cir_x - Circle_Center_X), 2) + pow((cir_y - Circle_Center_Y), 2) <= pow(Circle_outside_r, 2)) return 1;
    else
        return 0;
}
bool circle_outside(double cor_x, double cor_y)
{
    if (pow((cor_x - Circle_Center_X), 2) + pow((cor_y - Circle_Center_Y), 2) >= pow(Circle_inside_r, 2)) return 1;
    else
        return 0;
}

bool circle_inside_d(double cir_x, double cir_y)
{
    if (pow((cir_x - Circle_Center_X), 2) + pow((cir_y - Circle_Center_Y), 2) <= pow(Circle_outside_r + 1, 2)) return 1;
    else
        return 0;
}
bool circle_outside_d(double cor_x, double cor_y)
{
    if (pow((cor_x - Circle_Center_X), 2) + pow((cor_y - Circle_Center_Y), 2) >= pow(Circle_inside_r - 1, 2)) return 1;
    else
        return 0;
}

bool inside_circle_in(double ici_x, double ici_y)
{
    if (pow((ici_x - Circle_Center_X), 2) + pow((ici_y - Circle_Center_Y), 2) == pow(Circle_inside_r, 2)) return 1;
    else
        return 0;
}

bool outer_circle_in(double oci_x, double oci_y)
{
    if (pow((oci_x - Circle_Center_X), 2) + pow((oci_y - Circle_Center_Y), 2) == pow(Circle_outside_r, 2)) return 1;
    else
        return 0;
}

bool inside_circle_inside(double ici_x, double ici_y)
{
    if (pow((ici_x - Circle_Center_X), 2) + pow((ici_y - Circle_Center_Y), 2) < pow(Circle_inside_r, 2)) return 1;
    else
        return 0;
}

bool outer_circle_outer(double oci_x, double oci_y)
{
    if ((pow((oci_x - Circle_Center_X), 2) + pow((oci_y - Circle_Center_Y), 2)) > pow(Circle_outside_r, 2)) return 1;
    else
        return 0;
}

bool In_Rectangle(double IR_X, double IR_Y)
{
#if defined(Rectangle_Probe)
    if ((IR_X >= 209.837014 && IR_X <= 670.16299) && ((IR_Y >= 615 && IR_Y <= 815))) return 1;
    else
        return 0;
#else
    return 1;
#endif
}

void inside_circle_pos(double icp_y)
{
    sol1_i = sqrt(pow(Circle_inside_r, 2) - pow((icp_y - Circle_Center_Y), 2)) + Circle_Center_X;
    sol2_i = -sqrt(pow(Circle_inside_r, 2) - pow((icp_y - Circle_Center_Y), 2)) + Circle_Center_X;
}

void inside_circle_pos1(double icp_x)
{
    sol1_i = sqrt(pow(Circle_inside_r, 2) - pow((icp_x - Circle_Center_X), 2)) + Circle_Center_Y;
    sol2_i = -sqrt(pow(Circle_inside_r, 2) - pow((icp_x - Circle_Center_X), 2)) + Circle_Center_Y;
}

void outer_circle_pos(double ocp_y)
{
    sol1_o = sqrt(pow(Circle_outside_r, 2) - pow((ocp_y - Circle_Center_Y), 2)) + Circle_Center_X;
    sol2_o = -sqrt(pow(Circle_outside_r, 2) - pow((ocp_y - Circle_Center_Y), 2)) + Circle_Center_X;
}

void outer_circle_pos1(double ocp_x)
{
    sol1_o = sqrt(pow(Circle_outside_r, 2) - pow((ocp_x - Circle_Center_X), 2)) + Circle_Center_Y;
    sol2_o = -sqrt(pow(Circle_outside_r, 2) - pow((ocp_x - Circle_Center_X), 2)) + Circle_Center_Y;
}

double *inside_circle_posy_d(double icp_y)
{
    static double sol1[2] = {0};

    if (icp_y <= (Circle_Center_Y + Circle_inside_r) && icp_y >= (Circle_Center_Y - Circle_inside_r))
    {
        sol1[0] = sqrt(pow(Circle_inside_r, 2) - pow((icp_y - Circle_Center_Y), 2)) + Circle_Center_X;
        sol1[1] = -sqrt(pow(Circle_inside_r, 2) - pow((icp_y - Circle_Center_Y), 2)) + Circle_Center_X;

        return sol1;
    }

    else
        return error;
}

double *inside_circle_posx_d(double icp_x)
{
    static double sol2[2] = {0};

    if (icp_x <= (Circle_Center_X + Circle_inside_r) && icp_x >= (Circle_Center_X - Circle_inside_r))
    {
        sol2[0] = sqrt(pow(Circle_inside_r, 2) - pow((icp_x - Circle_Center_X), 2)) + Circle_Center_Y;
        sol2[1] = -sqrt(pow(Circle_inside_r, 2) - pow((icp_x - Circle_Center_X), 2)) + Circle_Center_Y;

        return sol2;
    }

    else
        return error;
}

double *outer_circle_posy_d(double ocp_y)
{
    static double sol3[2] = {0};

    if (ocp_y <= (Circle_Center_Y + Circle_outside_r) && ocp_y >= (Circle_Center_Y - Circle_outside_r))
    {
        sol3[0] = sqrt(pow(Circle_outside_r, 2) - pow((ocp_y - Circle_Center_Y), 2)) + Circle_Center_X;
        sol3[1] = -sqrt(pow(Circle_outside_r, 2) - pow((ocp_y - Circle_Center_Y), 2)) + Circle_Center_X;

        return sol3;
    }

    else
        return error;
}

double *outer_circle_posx_d(double ocp_x)
{
    static double sol4[2] = {0};

    if (ocp_x <= (Circle_Center_X + Circle_outside_r) && ocp_x >= (Circle_Center_X - Circle_outside_r))
    {
        sol4[0] = sqrt(pow(Circle_outside_r, 2) - pow((ocp_x - Circle_Center_X), 2)) + Circle_Center_Y;
        sol4[1] = -sqrt(pow(Circle_outside_r, 2) - pow((ocp_x - Circle_Center_X), 2)) + Circle_Center_Y;

        return sol4;
    }

    else
        return error;
}

void circle_print(void)
{
    FILE *pFile1;
    pFile1 = fopen("circle/circle_inside.txt", "w");

    printf("     ");
    for (int j = 0; j <= Circle_outside_r * 2; j += Mystep)
    {
        printf(" %3d ", j);
        if (Mystep <= 10)
            for (int jj = 0; jj < Mystep / 2.5; jj++)
                printf(" ");
    }
    printf("\n");

    for (int j = 0; j <= Circle_outside_r * 2; j += Mystep)
    {
        printf(" %3d ", j);
        for (int i = 0; i <= Circle_outside_r * 2; i += Mystep)
        {
            if ((circle_inside(i, j) && circle_outside(i, j)) == 1)
            {
                Mystep <= 10 ? printf("+") : printf("  +  ");
                fprintf(pFile1, "Y %3d X %3d\n", j, i);
            }

            else
                Mystep <= 10 ? printf("-") : printf("  -  ");
        }
        printf("\n");
    }
    printf(" %3d ", 870);
    for (int i = 0; i <= Circle_outside_r * 2; i += Mystep)
    {
        if ((circle_inside(i, 870) && circle_outside(i, 870)) == 1)
        {
            Mystep <= 10 ? printf("+") : printf("  +  ");
            fprintf(pFile1, "Y %3d X %3d\n", 870, i);
        }

        else
            Mystep <= 10 ? printf("-") : printf("  -  ");
    }
    printf("\n");
    printf("\n\n\n");

    fclose(pFile1);
}

void for_find_in_out_point(void)
{
    for (double j = 0; j <= region_max; j += Mystep)
    {
        if (j == region_max) j -= region_max - Circle_outside_r * 2;
        printf("Y:%.1f ", j);

        for (double i = 0; i <= region_max; i += Mystep)
        {
            if (i == region_max) i -= region_max - Circle_outside_r * 2;

            if (outer_circle_in(i, j) == 1) printf("Outer X:%.1lf ", i);
            if (inside_circle_in(i, j) == 1) printf("Inside X:%.1lf ", i);

            if (i == Circle_outside_r * 2) i += region_max - Circle_outside_r * 2;
        }
        printf("\n");
        if (j == Circle_outside_r * 2) j += region_max - Circle_outside_r * 2;
    }
    printf("\n");
}

void for_use_Y_find_X(void)
{
    FILE *pFile2;
    pFile2 = fopen("circle/circle_inside_inY.txt", "w");
    FILE *pFile3;
    pFile3 = fopen("circle/circle_outer_inY.txt", "w");

    for (double yy = 0; yy <= region_max; yy += Mystep)
    {
        if (yy == region_max) yy -= region_max - Circle_outside_r * 2;
        printf("\nY=%3.0lf  ", yy);
        inside_circle_pos(yy);
        if (sol1_i > 0 && sol2_i > 0)
        {
            printf("inside %.5lf %.5lf     ", sol1_i, sol2_i);
            fprintf(pFile2, "Y %3.0lf %3.5lf %3.5lf\n", yy, sol1_i, sol2_i);
        }

        else
            printf("                               ");
        outer_circle_pos(yy);
        if (sol1_o > 0 && sol2_o > 0)
        {
            printf("outer  %.5lf %.5lf", sol1_o, sol2_o);
            fprintf(pFile3, "Y %3.0lf %3.5lf %3.5lf\n", yy, sol1_o, sol2_o);
        }
        if (yy == Circle_outside_r * 2) yy += region_max - Circle_outside_r * 2;
    }
    printf("\n");

    fclose(pFile2);
    fclose(pFile3);
}

void use_x_find_y_inter(double xfy_i)
{
    temp = inside_circle_posx_d(xfy_i);

    if (temp[0] > 0 && temp[1] > 0)
    {
        printf("inside %.5lf %.5lf     ", temp[0], temp[1]);
    }
}

void for_use_X_find_Y(void)
{
    FILE *pFile4;
    pFile4 = fopen("circle/circle_inside_inX.txt", "w");
    FILE *pFile5;
    pFile5 = fopen("circle/circle_outer_inX.txt", "w");

    for (double xx = 0; xx <= region_max; xx += Mystep)
    {
        if (xx == region_max) xx -= region_max - Circle_outside_r * 2;

        printf("\nX=%3.0lf  ", xx);
        inside_circle_pos1(xx);
        if (sol1_i > 0 && sol2_i > 0)
        {
            printf("inside %.5lf %.5lf     ", sol1_i, sol2_i);
            fprintf(pFile4, "X %3.0lf %3.5lf %3.5lf\n", xx, sol1_i, sol2_i);
        }
        else
            printf("                               ");

        outer_circle_pos1(xx);
        if (sol1_o > 0 && sol2_o > 0)
        {
            printf("outer  %.5lf %.5lf ", sol1_o, sol2_o);
            fprintf(pFile5, "X %3.0lf %3.5lf %3.5lf\n", xx, sol1_o, sol2_o);
        }

        if (xx == Circle_outside_r * 2) xx += region_max - Circle_outside_r * 2;
    }
    printf("\n");

    fclose(pFile4);
    fclose(pFile5);
}

bool find_region_in_out(double x_fr, double y_fr)
{
#if defined(Rectangle_Probe)
    if ((circle_inside(x_fr, y_fr) && circle_outside(x_fr, y_fr) && In_Rectangle(x_fr, y_fr)) == 1) return 1;
    else
        return 0;
#else
    if ((circle_inside(x_fr, y_fr) && circle_outside(x_fr, y_fr)) == 1) return 1;
    else
        return 0;
#endif
}

bool find_region_in_out_d(double x_fr, double y_fr)
{
#if defined(Rectangle_Probe)
    if ((circle_inside_d(x_fr, y_fr) && circle_outside_d(x_fr, y_fr)) == 1) return 1;
    else
        return 0;
#else
    if ((circle_inside_d(x_fr, y_fr) && circle_outside_d(x_fr, y_fr)) == 1) return 1;
    else
        return 0;
#endif
}

void print_012(void)
{
    for (double j = region_max; j >= 0; j -= Mystep)
    {
        if (j == region_max) j -= region_max - Circle_outside_r * 2;
        printf(" %3d | ", (int)j);
        for (double i = region_max; i >= 0; i -= Mystep)
        {
            if (i == region_max) i -= region_max - Circle_outside_r * 2;

            temp = outer_circle_posx_d(i);
            if (outer_circle_outer(i, j) == 1) printf("1 ");
            else if (inside_circle_inside(i, j) == 1)
                printf("2 ");
            else
                printf("0 ");

            if (i == Circle_outside_r * 2) i += region_max - Circle_outside_r * 2;
        }
        printf("\n");
        if (j == Circle_outside_r * 2) j += region_max - Circle_outside_r * 2;
    }
}

void print_point(void)
{
    FILE *pFILE;
    FILE *pFILE1;
    FILE *pFILE2;
    FILE *pFILE2_special;
    FILE *pFILE_ALL;
    FILE *pFILE_ALL_ADD;
    pFILE = fopen("circle/circle_XYZ.txt", "w");
    pFILE1 = fopen("circle/circle_XYZ_in.txt", "w");
    pFILE2 = fopen("circle/circle_XYZ_out.txt", "w");
    pFILE2_special = fopen("circle/circle_XYZ_out_special.txt", "w");
    pFILE_ALL = fopen("circle/circle_XYZ_ALL.txt", "w");
    pFILE_ALL_ADD = fopen("circle/circle_XYZ_ALL_ADD.txt", "w");
    char Format[] = "%.5lf %.5lf\n"; //" { % .5lf, % .5lf, 0},\n ";
    char Format_mat[] = "{%.5lf, %.5lf, 0},\n";
    char FormatY[] = "X %.5lf Y %.5lf\n";
    char FormatX[] = "Y %.5lf X %.5lf\n";

    for (double j = 0; j <= region_max; j += Mystep) //(double j = region_max; j >= 0; j -= Mystep)
    {
        if (j == region_max) j -= region_max - Circle_outside_r * 2;
        printf(" %3d | ", (int)j);
        for (double i = 0; i <= region_max; i += Mystep)
        {
            if (i == region_max) i -= region_max - Circle_outside_r * 2;
            // total = (()j / 10) + (i / 100);
            total = (j == Circle_outside_r * 2) ? ((j + 20) / 10) : (j / 10);
            total += (i == Circle_outside_r * 2) ? ((i + 20) / 100) : (i / 100);
            // printf("%02d ", total);
            temp = outer_circle_posx_d(i);   // x find y
            temp1 = outer_circle_posy_d(j);  // y find x
            temp2 = inside_circle_posx_d(i); // x find y
            temp3 = inside_circle_posy_d(j); // y find x

            if (find_region_in_out(i, j) == 0)
            {
                if ((j <= 100 || j >= 800) && (i >= 200 && i <= 700) && total != 87)
                {
                    printf(" OYO");
                    fprintf(pFILE2, FormatY, i, (j >= Circle_outside_r) ? temp[0] : temp[1]);
                    fprintf(pFILE_ALL, Format_mat, i, (j >= Circle_outside_r) ? temp[0] : temp[1]);
                }
                else if ((j <= 700 && j >= 200) && (i < 100 || i > 800) && (total != 79))
                {
                    printf(" OXO");
                    fprintf(pFILE2, FormatX, j, (i >= Circle_outside_r) ? temp1[0] : temp1[1]);
                    fprintf(pFILE_ALL, Format_mat, (i >= Circle_outside_r) ? temp1[0] : temp1[1], j);
                }
                else
                {
                    if ((total == 11 || total == 18 || total == 81 || total == 87 || total == 78) && (i != Circle_outside_r * 2))
                    {
                        printf(" SSS");
                        fprintf(pFILE2, FormatY, i, (j >= Circle_outside_r) ? temp[0] : temp[1]);
                        fprintf(pFILE_ALL_ADD, Format_mat, (i >= Circle_outside_r) ? temp1[0] : temp1[1], j);
                        fprintf(pFILE2_special, FormatY, (i >= Circle_outside_r) ? temp1[0] : temp1[1], j);
                        fprintf(pFILE_ALL, Format_mat, i, (j >= Circle_outside_r) ? temp[0] : temp[1]);
                    }
                    else if (circle_inside(i, j) == 1 && (total != 44 && total != 45 && total != 54 && total != 55))
                    {
                        printf(" IIC");
                        if (total == 34 || total == 35 || total == 64 || total == 65)
                        {
                            fprintf(pFILE1, FormatY, i, (j >= Circle_outside_r) ? temp2[0] : temp2[1]);
                            fprintf(pFILE1, FormatX, j, (i >= Circle_outside_r) ? temp3[0] : temp3[1]);

                            fprintf(pFILE_ALL, Format_mat, i, (j >= Circle_outside_r) ? temp2[0] : temp2[1]);
                            fprintf(pFILE_ALL_ADD, Format_mat, (i >= Circle_outside_r) ? temp3[0] : temp3[1], j);
                        }
                        else
                        {
                            fprintf(pFILE1, FormatX, j, (i >= Circle_outside_r) ? temp3[0] : temp3[1]);
                            fprintf(pFILE1, FormatY, i, (j >= Circle_outside_r) ? temp2[0] : temp2[1]);

                            fprintf(pFILE_ALL, Format_mat, i, (j >= Circle_outside_r) ? temp2[0] : temp2[1]);
                            fprintf(pFILE_ALL_ADD, Format_mat, (i >= Circle_outside_r) ? temp3[0] : temp3[1], j);
                        }
                    }
                    else
                    {
                        printf(" XXX");
                        fprintf(pFILE_ALL, Format_mat, i, j);
                    }
                }
            }
            else
            {
                if (i == 0)
                {
                    printf(" 000");
                    fprintf(pFILE, Format, i, j, 0);
                    fprintf(pFILE_ALL, Format_mat, i, j);
                }
                else
                {
                    printf(" %3.0lf", i);
                    fprintf(pFILE, Format, i, j, 0);
                    fprintf(pFILE_ALL, Format_mat, i, j);
                }
            }

            if (i == Circle_outside_r * 2) i += region_max - Circle_outside_r * 2;
        }
        printf("\n");
        if (j == Circle_outside_r * 2) j += region_max - Circle_outside_r * 2;
    }
    fclose(pFILE);
    fclose(pFILE1);
    fclose(pFILE2);
    fclose(pFILE2_special);
}

void Produce_Mesh(void)
{
    FILE *pFILE;
    pFILE = fopen("circle/circle_Mesh.txt", "w");
    FILE *pFILE1;
    pFILE1 = fopen("circle/circle_Mesh1.txt", "w");

    printf("\nProduce circle_mesh.txt and print out\n");

#if !defined(Rectangle_Probe)
    printf("      ");
    for (int i = 0; i <= region_max; i += Mystep)
        printf("%-3d  ", (i == region_max) ? Circle_outside_r * 2 : i);
    printf("\n");
#endif

/*
for (double j = 0; j <= region_max; j += Mystep) //(double j = region_max; j >= 0; j -= Mystep)
{
    if (j == region_max) j -= region_max - Circle_outside_r * 2;
    printf("%3d | ", (int)j);
    for (double i = 0; i <= region_max; i += Mystep)
    {
        if (i == region_max) i -= region_max - Circle_outside_r * 2;

        if (i == Circle_outside_r * 2) i += region_max - Circle_outside_r * 2;
    }
    printf("\n");
    if (j == Circle_outside_r * 2) j += region_max - Circle_outside_r * 2;
}
printf("\n");
*/
#if defined(Rectangle_Probe)
    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 10; i++)
        {
            int total = j * 10 + i;

            if (In_Rectangle(XYZ_Position[total][0], XYZ_Position[total][1]))
            {
                double data_temp = (double)(rand() % 10) / 100;
                fprintf(pFILE, "%.2lf ", data_temp);
                printf("%.2lf ", data_temp);
            }
            else
            {
                fprintf(pFILE, "0.00 ");
                printf("XXX  ");
            }
        }
        fprintf(pFILE, "\n");
        printf("\n");
    }

#else
    for (int j = 0; j <= region_max; j += Mystep)
    {
        if (j == region_max) j -= region_max - Circle_outside_r * 2;
        printf("%-3d | ", j);
        for (int i = 0; i <= 900; i += Mystep)
        {
            total = (int)(((j == Circle_outside_r * 2) ? ((j + 20) / 10) : j / 10) + (i / 100));
            double data_temp = (double)(rand() % 10) / 100;
            if (j == region_max) j -= region_max - Circle_outside_r * 2;

            if (total == 0 || total == 1 || total == 8 || total == 9 || total == 10 || total == 19 || total == 79 || total == 80 || total == 88 ||
                total == 89 || total == 90 || total == 97 || total == 98 || total == 99 || total == 44 || total == 45 || total == 54 || total == 55)
            {
                fprintf(pFILE, "0.00 ");
                printf("0.00 ");
            }
            else
            {
                fprintf(pFILE, "%.2lf ", data_temp);
                printf("%.2lf ", data_temp);
            }

            if (j == Circle_outside_r * 2) j += region_max - Circle_outside_r * 2;
        }
        fprintf(pFILE, "\n");
        printf("\n");
        if (j == Circle_outside_r * 2) j += region_max - Circle_outside_r * 2;
    }

    printf("\nProduce circle_mesh1.txt and print out\n");

    for (int i = 0; i < 13; i++)
    {
        double data_temp = (double)(rand() % 10) / 100;

        if ((i != 0) && (i % 5 == 0))
        {
            fprintf(pFILE1, "\n");
            printf("\n");
        }

        fprintf(pFILE1, "%.2lf ", data_temp);
        printf("%.2lf ", data_temp);
    }
#endif

    printf("\n");
    fclose(pFILE);
    fclose(pFILE1);
}

void Load_Mesh(double z_offset_temp)
{
    FILE *pFILE;
    std::string path = ros::package::getPath("gcode_translation");
    path += "/include/Mesh/circle/circle_Mesh.txt";
    pFILE = fopen(path.c_str(), "r");
#if !defined(Rectangle_Probe)
    FILE *pFILE1;
    pFILE1 = fopen("circle/circle_Mesh1.txt", "r");
#endif

    for (int j = 9; j >= 0; j--)
    {
        if (j == 8 || j == 6)
            fscanf(pFILE, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", &XYZ_Position[j * 10 + 9][2], &XYZ_Position[j * 10 + 8][2],
                   &XYZ_Position[j * 10 + 7][2], &XYZ_Position[j * 10 + 6][2], &XYZ_Position[j * 10 + 5][2], &XYZ_Position[j * 10 + 4][2],
                   &XYZ_Position[j * 10 + 3][2], &XYZ_Position[j * 10 + 2][2], &XYZ_Position[j * 10 + 1][2], &XYZ_Position[j * 10][2]);
        else
            fscanf(pFILE, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", &XYZ_Position[j * 10][2], &XYZ_Position[j * 10 + 1][2],
                   &XYZ_Position[j * 10 + 2][2], &XYZ_Position[j * 10 + 3][2], &XYZ_Position[j * 10 + 4][2], &XYZ_Position[j * 10 + 5][2],
                   &XYZ_Position[j * 10 + 6][2], &XYZ_Position[j * 10 + 7][2], &XYZ_Position[j * 10 + 8][2], &XYZ_Position[j * 10 + 9][2]);
    }

    for (int i = 0; i < 100; i++)
    {
        if (In_Rectangle(XYZ_Position[i][0], XYZ_Position[i][1]))
        {
            XYZ_Position[i][2] = XYZ_Position[i][2] - z_offset_temp;
        }
    }

#if !defined(Rectangle_Probe)
    for (int j = 0; j < 3; j++)
    {
        if (j == 2) fscanf(pFILE1, "%lf %lf %lf", &XYZ_Position1_Add[10][2], &XYZ_Position1_Add[11][2], &XYZ_Position1_Add[12][2]);
        else
            fscanf(pFILE1, "%lf %lf %lf %lf %lf\n", &XYZ_Position1_Add[j * 5][2], &XYZ_Position1_Add[j * 5 + 1][2], &XYZ_Position1_Add[j * 5 + 2][2],
                   &XYZ_Position1_Add[j * 5 + 3][2], &XYZ_Position1_Add[j * 5 + 4][2]);
    }
#endif

    printf("\ncircle_Mesh.txt\n      ");
    for (int i = 0; i <= region_max; i += Mystep)
        printf("%-3d  ", (i == region_max) ? (i - (region_max - Circle_outside_r * 2)) : i);
    printf("\n");
    for (int j = 0; j < 10; j += 1)
    {
        printf("%-3d | ", (j != 9) ? j * 100 : j * 100 - 20);
        for (int i = 0; i < 10; i += 1)
        {
            printf("%-3.2lf ", (double)XYZ_Position[j * 10 + i][2]);
        }
        printf("\n");
    }
#if !defined(Rectangle_Probe)
    printf("\ncircle_Mesh1.txt");
    for (int i = 0; i < 13; i++)
    {
        if (i % 5 == 0) printf("\n");

        printf("%-3.2lf ", (double)XYZ_Position1_Add[i][2]);
    }
    printf("\n");
#endif

    fclose(pFILE);
#if !defined(Rectangle_Probe)
    fclose(pFILE1);
#endif
    printf("\nRectangle probe points\n");

#if defined(Print_Rectangle)
    printf("   -------------------------------------------------\n");
    for (int j = 9; j >= 0; j--)
    {
        for (int i = 0; i < 10; i++)
        {
            if (In_Rectangle(XYZ_Position[j * 10 + i][0], XYZ_Position[j * 10 + i][1]))
            {
                if (i == 2) printf("   | %.0lf | ", XYZ_Position[j * 10 + i][1]);
                printf("%.2lf | ", XYZ_Position[j * 10 + i][2]);
                if (i == 7) printf("\n");
            }
        }
    }
    printf("   |-----|------------------------------------------\n");
    printf("   |     | ");
    for (int i = 0; i < 6; i++)
    {
        printf("%.0lf  | ", XYZ_Position[62 + i][0]);
    }
    printf("\n   -------------------------------------------------\n");
    printf("\n");
#endif
}

void Calc_abcd(void)
{
    printf("\n");
    int counter = 0;
    for (int j = 0; j < 9; j += 1)
    {
        for (int i = 0; i < 9; i += 1)
        {
            total = j * 10 + i;

#if defined(Rectangle_Probe)
            if (In_Rectangle(XYZ_Position[total][0], XYZ_Position[total][1]) && XYZ_Position[total][0] < 670 && XYZ_Position[total][1] < 815)
            {
                for (int ii = 0; ii < 3; ii++)
                {
                    differ_ab_1[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + i][ii];
                    differ_ac_1[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii];

                    differ_ab_2[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                    differ_ac_2[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                }
    #if defined(Print_Detail)
                printf("ab1[%d]={%.5lf, %.5lf, %.5lf}, ac1[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                       differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
                printf("ab2[%d]={%.5lf, %.5lf, %.5lf}, ac2[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_2[total][0], differ_ab_2[total][1],
                       differ_ab_2[total][2], total, differ_ac_2[total][0], differ_ac_2[total][1], differ_ac_2[total][2]);
    #endif
            }
            else
            {
                for (int ii = 0; ii < 3; ii++)
                {
                    differ_ab_1[total][ii] = 0;
                    differ_ac_1[total][ii] = 0;

                    differ_ab_2[total][ii] = 0;
                    differ_ac_2[total][ii] = 0;
                }
    #if defined(Print_Detail)
                printf("ab1[%d]={%.5lf, %.5lf, %.5lf}, ac1[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                       differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
                printf("ab2[%d]={%.5lf, %.5lf, %.5lf}, ac2[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_2[total][0], differ_ab_2[total][1],
                       differ_ab_2[total][2], total, differ_ac_2[total][0], differ_ac_2[total][1], differ_ac_2[total][2]);
    #endif
            }
#else
            if (total == 0 || total == 8 || total == 78 || total == 80 || total == 87 || total == 88 || total == 34 || total == 43 || total == 44 ||
                total == 45 || total == 54) // out region
                counter++;                  // printf(" XXX ");

            else if (total == 1 || total == 7 || total == 10 || total == 18 || total == 68 || total == 70 || total == 77 || total == 81 ||
                     total == 86)
            { // Outside the circle 3 points
                counter++;
                // printf("%.2lf ", (double)XYZ_Position[j * 10 + i][2]);

                if ((j <= Circle_outside_r / 100) && (i <= Circle_outside_r / 100))
                {
                    for (int ii = 0; ii < 3; ii++)
                    {
                        differ_ab_1[total][ii] = (total == 1) ? XYZ_Position1_Add[0][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii]
                                                              : XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                        differ_ac_1[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                    }
    #if defined(Print_Detail)

                    printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                           differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);

    #endif
                }
                else if ((j <= Circle_outside_r / 100) && (i > Circle_outside_r / 100))
                {
                    for (int ii = 0; ii < 3; ii++)
                    {
                        differ_ab_1[total][ii] = (total == 7) ? XYZ_Position1_Add[1][ii] - XYZ_Position[((j + 1) * 10) + i][ii]
                                                              : XYZ_Position[(j + 1) * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + i][ii];
                        differ_ac_1[total][ii] = XYZ_Position[j * 10 + i][ii] - XYZ_Position[(j + 1) * 10 + i][ii];
                    }
    #if defined(Print_Detail)
                    printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                           differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
    #endif
                }
                else if ((j > Circle_outside_r / 100) && (i <= Circle_outside_r / 100))
                {
                    for (int ii = 0; ii < 3; ii++)
                    {
                        differ_ab_1[total][ii] = (total == 81) ? XYZ_Position1_Add[11][ii] - XYZ_Position[j * 10 + (i + 1)][ii]
                                                               : XYZ_Position[j * 10 + i][ii] - XYZ_Position[j * 10 + (i + 1)][ii];
                        differ_ac_1[total][ii] = XYZ_Position[(j + 1) * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + (i + 1)][ii];
                    }
    #if defined(Print_Detail)
                    printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                           differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
    #endif
                }
                else
                {
                    for (int ii = 0; ii < 3; ii++)
                    {
                        differ_ab_1[total][ii] = (total == 86) ? XYZ_Position1_Add[12][ii] - XYZ_Position[j * 10 + i][ii]
                                                               : (total == 77) ? XYZ_Position1_Add[10][ii] - XYZ_Position[j * 10 + i][ii]
                                                                               : XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + i][ii];
                        differ_ac_1[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii];
                    }
    #if defined(Print_Detail)
                    printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                           differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
    #endif
                }
            }

            else if (total == 2 || total == 3 || total == 5 || total == 6 || total == 20 || total == 28 || total == 30 || total == 38 ||
                     total == 50 || total == 58 || total == 60 || total == 82 || total == 83 || total == 85 || total == 4 || total == 84 ||
                     total == 48 || total == 40)
            { // Outside the circle 4 points
                counter++;
                for (int ii = 0; ii < 3; ii++)
                {
                    differ_ab_1[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + i][ii];
                    differ_ac_1[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii];

                    differ_ab_2[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                    differ_ac_2[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                }
    #if defined(Print_Detail)
                printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                       differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
                printf("ab2[%d]={%.5lf, %.5lf, %.5lf}, ac2[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_2[total][0], differ_ab_2[total][1],
                       differ_ab_2[total][2], total, differ_ac_2[total][0], differ_ac_2[total][1], differ_ac_2[total][2]);
    #endif
            }

            else if (total == 12 || total == 13 || total == 14 || total == 15 || total == 16 || total == 21 || total == 22 || total == 26 ||
                     total == 27 || total == 31 || total == 37 || total == 41 || total == 47 || total == 51 || total == 56 || total == 57 ||
                     total == 61 || total == 62 || total == 65 || total == 66 || total == 72 || total == 73 || total == 74 || total == 75)
            { // in region
                counter++;
                for (int ii = 0; ii < 3; ii++)
                {
                    differ_ab_1[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + i][ii];
                    differ_ac_1[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii];

                    differ_ab_2[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                    differ_ac_2[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                }
    #if defined(Print_Detail)
                printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                       differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
                printf("ab2[%d]={%.5lf, %.5lf, %.5lf}, ac2[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_2[total][0], differ_ab_2[total][1],
                       differ_ab_2[total][2], total, differ_ac_2[total][0], differ_ac_2[total][1], differ_ac_2[total][2]);
    #endif
            }

            else if (total == 33 || total == 35 || total == 53 || total == 55)
            { // in the inner circle 3 point
                counter++;
                for (int ii = 0; ii < 3; ii++)
                {
                    int temp_3 = (total == 33) ? 2 : (total == 35) ? 3 : (total == 53) ? 8 : 9;
                    differ_ab_1[total][ii] =
                        (i <= (Circle_outside_r - 100) / 100)
                            ? (j <= (Circle_outside_r - 100) / 100) ? XYZ_Position1_Add[temp_3][ii] - XYZ_Position[j * 10 + i][ii]
                                                                    : XYZ_Position1_Add[temp_3][ii] - XYZ_Position[(j + 1) * 10 + i][ii]
                            : (j <= (Circle_outside_r - 100) / 100) ? XYZ_Position1_Add[temp_3][ii] - XYZ_Position[j * 10 + (i + 1)][ii]
                                                                    : XYZ_Position1_Add[temp_3][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                    differ_ac_1[total][ii] =
                        (j <= (Circle_outside_r - 100) / 100)
                            ? (i <= (Circle_outside_r - 100) / 100) ? XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii]
                                                                    : XYZ_Position[(j + 1) * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + (i + 1)][ii]
                            : (i <= (Circle_outside_r - 100) / 100) ? XYZ_Position[j * 10 + i][ii] - XYZ_Position[(j + 1) * 10 + i][ii]
                                                                    : XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                }
    #if defined(Print_Detail)
                printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                       differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
    #endif
            }

            else if (total == 24 || total == 42 || total == 46 || total == 64)
            { // in the inner circle 4 point
                counter++;
                for (int ii = 0; ii < 3; ii++)
                {
                    differ_ab_1[total][ii] = (total == 42) ? XYZ_Position1_Add[4][ii] - XYZ_Position[j * 10 + i][ii]
                                                           : (total == 46) ? XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position1_Add[5][ii]
                                                                           : XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + i][ii];
                    differ_ac_1[total][ii] = (total == 42) ? XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii]
                                                           : (total == 46) ? XYZ_Position1_Add[7][ii] - XYZ_Position1_Add[5][ii]
                                                                           : XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii];

                    differ_ab_2[total][ii] = (total == 42) ? XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position1_Add[6][ii]
                                                           : (total == 46) ? XYZ_Position1_Add[7][ii] - XYZ_Position[(j + 1) * 10 + i][ii]
                                                                           : XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + i][ii];
                    differ_ac_2[total][ii] = (total == 42) ? XYZ_Position1_Add[4][ii] - XYZ_Position1_Add[6][ii]
                                                           : XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                }
    #if defined(Print_Detail)
                printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                       differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
                printf("ab2[%d]={%.5lf, %.5lf, %.5lf}, ac2[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_2[total][0], differ_ab_2[total][1],
                       differ_ab_2[total][2], total, differ_ac_2[total][0], differ_ac_2[total][1], differ_ac_2[total][2]);
    #endif
            }

            else if (total == 11 || total == 17 || total == 71 || total == 76 || total == 67)
            { // Outside circle 5 point -> 4 point
                counter++;

                for (int ii = 0; ii < 3; ii++)
                {
                    // XYZ_Position1_5to4[][]s
                    differ_ab_1[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + i][ii];
                    differ_ac_1[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii];

                    differ_ab_2[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                    differ_ac_2[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                }
    #if defined(Print_Detail)
                printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                       differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
                printf("ab2[%d]={%.5lf, %.5lf, %.5lf}, ac2[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_2[total][0], differ_ab_2[total][1],
                       differ_ab_2[total][2], total, differ_ac_2[total][0], differ_ac_2[total][1], differ_ac_2[total][2]);
    #endif
            }

            else if (total == 23 || total == 25 || total == 32 || total == 36 || total == 52 || total == 63 || total == 65 || total == 56)
            { // in the inner circle 5 point -> 4 point
                counter++;

                for (int ii = 0; ii < 3; ii++)
                {
                    differ_ab_1[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[j * 10 + i][ii];
                    differ_ac_1[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[j * 10 + i][ii];

                    differ_ab_2[total][ii] = XYZ_Position[(j + 1) * 10 + i][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                    differ_ac_2[total][ii] = XYZ_Position[j * 10 + (i + 1)][ii] - XYZ_Position[(j + 1) * 10 + (i + 1)][ii];
                }

    #if defined(Print_Detail)
                printf("ab[%d]={%.5lf, %.5lf, %.5lf}, ac[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_1[total][0], differ_ab_1[total][1],
                       differ_ab_1[total][2], total, differ_ac_1[total][0], differ_ac_1[total][1], differ_ac_1[total][2]);
                printf("ab2[%d]={%.5lf, %.5lf, %.5lf}, ac2[%d]={%.5lf, %.5lf, %.5lf}\n", total, differ_ab_2[total][0], differ_ab_2[total][1],
                       differ_ab_2[total][2], total, differ_ac_2[total][0], differ_ac_2[total][1], differ_ac_2[total][2]);
    #endif
            }
            else
            {
                counter++;
                printf("%2d \n", total);
            }
            // printf("%2d ", total);
#endif
        }
        // printf("\n");
    }

    for (int j = 0; j < 9; j++)
    {
        for (int i = 0; i < 9; i++)
        {
            a_1[j * 10 + i] = differ_ab_1[j * 10 + i][1] * differ_ac_1[j * 10 + i][2] - differ_ab_1[j * 10 + i][2] * differ_ac_1[j * 10 + i][1];
            b_1[j * 10 + i] = -differ_ab_1[j * 10 + i][0] * differ_ac_1[j * 10 + i][2] + differ_ab_1[j * 10 + i][2] * differ_ac_1[j * 10 + i][0];
            c_1[j * 10 + i] = differ_ab_1[j * 10 + i][0] * differ_ac_1[j * 10 + i][1] - differ_ab_1[j * 10 + i][1] * differ_ac_1[j * 10 + i][0];

            a_2[j * 10 + i] = differ_ab_2[j * 10 + i][1] * differ_ac_2[j * 10 + i][2] - differ_ab_2[j * 10 + i][2] * differ_ac_2[j * 10 + i][1];
            b_2[j * 10 + i] = -differ_ab_2[j * 10 + i][0] * differ_ac_2[j * 10 + i][2] + differ_ab_2[j * 10 + i][2] * differ_ac_2[j * 10 + i][0];
            c_2[j * 10 + i] = differ_ab_2[j * 10 + i][0] * differ_ac_2[j * 10 + i][1] - differ_ab_2[j * 10 + i][1] * differ_ac_2[j * 10 + i][0];
        }
    }

    for (int j = 0; j < 9; j++)
    {
        for (int i = 0; i < 9; i++)
        {
            total = j * 10 + i;
// printf(" %d, ", total);
#if defined(Rectangle_Probe)
            d_1[j * 10 + i] = -a_1[j * 10 + i] * XYZ_Position[j * 10 + i][0] - b_1[j * 10 + i] * XYZ_Position[j * 10 + i][1] -
                              c_1[j * 10 + i] * XYZ_Position[j * 10 + i][2];
            d_2[j * 10 + i] = -a_2[j * 10 + i] * XYZ_Position[(j + 1) * 10 + (i + 1)][0] - b_2[j * 10 + i] * XYZ_Position[(j + 1) * 10 + (i + 1)][1] -
                              c_2[j * 10 + i] * XYZ_Position[(j + 1) * 10 + (i + 1)][2];
#else
            if (total == 1 || total == 10)
            {
                d_1[j * 10 + i] = -a_1[j * 10 + i] * XYZ_Position[(j + 1) * 10 + (i + 1)][0] -
                                  b_1[j * 10 + i] * XYZ_Position[(j + 1) * 10 + (i + 1)][1] -
                                  c_1[j * 10 + i] * XYZ_Position[(j + 1) * 10 + (i + 1)][2];
            }
            else if (total == 7 || total == 18)
            {
                d_1[j * 10 + i] = -a_1[j * 10 + i] * XYZ_Position[(j + 1) * 10 + i][0] - b_1[j * 10 + i] * XYZ_Position[(j + 1) * 10 + i][1] -
                                  c_1[j * 10 + i] * XYZ_Position[(j + 1) * 10 + i][2];
            }
            else if (total == 70 || total == 81)
            {
                d_1[j * 10 + i] = -a_1[j * 10 + i] * XYZ_Position[j * 10 + (i + 1)][0] - b_1[j * 10 + i] * XYZ_Position[j * 10 + (i + 1)][1] -
                                  c_1[j * 10 + i] * XYZ_Position[j * 10 + (i + 1)][2];
            }
            else if (total == 68 || total == 86 || total == 77)
            {
                d_1[j * 10 + i] = -a_1[j * 10 + i] * XYZ_Position[j * 10 + i][0] - b_1[j * 10 + i] * XYZ_Position[j * 10 + i][1] -
                                  c_1[j * 10 + i] * XYZ_Position[j * 10 + i][2];
            }
            else
            {
                d_1[j * 10 + i] = -a_1[j * 10 + i] * XYZ_Position[j * 10 + (i + 1)][0] - b_1[j * 10 + i] * XYZ_Position[j * 10 + (i + 1)][1] -
                                  c_1[j * 10 + i] * XYZ_Position[j * 10 + (i + 1)][2];
            }
            d_2[j * 10 + i] = -a_2[j * 10 + i] * XYZ_Position[j * 10 + i][0] - b_2[j * 10 + i] * XYZ_Position[j * 10 + (i + 11)][1] -
                              c_2[j * 10 + i] * XYZ_Position[j * 10 + i][2];
#endif
        }
    }

    /*
    for (int j = 0; j < 9; j++)
    {
        for (int i = 0; i < 9; i++)
        {
            printf("Region: %2d ", j * 10 + i);
            printf("    a1:%.5lf b1:%.5lf c1:%.5lf d1:%.5lf\n", a_1[j * 10 + i], b_1[j * 10 + i], c_1[j * 10 + i], d_1[j * 10 + i]);
            printf("               a2:%.5lf b2:%.5lf c2:%.5lf d2:%.5lf\n", a_2[j * 10 + i], b_2[j * 10 + i], c_2[j * 10 + i], d_2[j * 10 + i]);
        }
    }
    //*/

    printf("counter:%d\n", counter);
}

//*/
double calc_z(double calc_x, double calc_y)
{
    if (find_region_in_out(calc_x, calc_y))
    {
        double z_pos_temp = 0;
        double x_diff, y_diff;
        int x_div = (int)((int)calc_x / 100);
        int y_div = (int)((int)calc_y / 100);
        double slope = 0, k = 0;
        if (x_div < 0) x_div = 0;
        if (y_div < 0) y_div = 0;
        slope = (XYZ_Position[(y_div + 1) * 10 + x_div][1] - XYZ_Position[y_div * 10 + (x_div + 1)][1]) /
                (XYZ_Position[(y_div + 1) * 10 + x_div][0] - XYZ_Position[y_div * 10 + (x_div + 1)][0]);
        k = XYZ_Position[(y_div + 1) * 10 + x_div][1] - slope * XYZ_Position[(y_div + 1) * 10 + x_div][0];

        x_diff = calc_x - x_div * 100;
        y_diff = calc_y - y_div * 100;

//*
#if defined(Print_Detail)
        printf("X pos: %.5lf Y pos: %.5lf\n", calc_x, calc_y);
        printf("slope: %.5lf k: %.5lf\n", slope, k);
        printf("x_div: %d, y_div: %d\n", x_div, y_div);

        printf("a1: %.5lf b1: %.5lf c1: %.5lf d1: %.5lf\n", a_1[y_div * 10 + x_div], b_1[y_div * 10 + x_div], c_1[y_div * 10 + x_div],
               d_1[y_div * 10 + x_div]);
        printf("a2: %.5lf b2: %.5lf c2: %.5lf d2: %.5lf\n", a_2[y_div * 10 + x_div], b_2[y_div * 10 + x_div], c_2[y_div * 10 + x_div],
               d_2[y_div * 10 + x_div]);
        printf("z_pos1: %lf z_pos2: %lf\n",
               (-d_1[y_div * 10 + x_div] - a_1[y_div * 10 + x_div] * calc_x - b_1[y_div * 10 + x_div] * calc_y) / c_1[y_div * 10 + x_div],
               (-d_2[y_div * 10 + x_div] - a_2[y_div * 10 + x_div] * calc_x - b_2[y_div * 10 + x_div] * calc_y) / c_2[y_div * 10 + x_div]);
#endif

        //*/
        // printf("Z real: %.5lf\n", XYZ_Position[y_div * 10 + x_div][2]);
        if (((-slope * calc_x + calc_y) <= k) || (a_2[y_div * 10 + x_div] == 0 && a_2[y_div * 10 + x_div] == 0 && b_2[y_div * 10 + x_div] == 0 &&
                                                  c_2[y_div * 10 + x_div] == 0 && d_2[y_div * 10 + x_div] == 0))
        {
            z_pos_temp = (-d_1[y_div * 10 + x_div] - a_1[y_div * 10 + x_div] * calc_x - b_1[y_div * 10 + x_div] * calc_y) / c_1[y_div * 10 + x_div];
        }
        else
        {
            z_pos_temp = (-d_2[y_div * 10 + x_div] - a_2[y_div * 10 + x_div] * calc_x - b_2[y_div * 10 + x_div] * calc_y) / c_2[y_div * 10 + x_div];
        }
        printf("%.5lf\n", z_pos_temp);
        return z_pos_temp;
    }
    else
        return 9.99;
}

#if defined(Rectangle_Probe)
double calc_z_rectangle(double calc_x, double calc_y, bool choice)
{
    switch (choice)
    {
        case true:
    #if defined(Print_Detail)
            printf("Before Offset X: %.5lf m Y: %.5lf m\n", calc_x, calc_y);
    #endif
            // Unit: m -> mm
            calc_x = calc_x * 1000;
            calc_y = calc_y * 1000;
            // Center (0,0) -> (440,400)
            calc_x = calc_x + Circle_outside_r;
            calc_y = calc_y + Circle_outside_r;
    #if defined(Print_Detail)
            printf("After Offset X: %.5lf mm Y: %.5lf mm\n", calc_x, calc_y);
    #endif

            break;
        case false:
    #if defined(Print_Detail)
            printf("No offset needed!\n");
    #endif
            break;
    }

    if (In_Rectangle(calc_x, calc_y))
    {
        double z_pos_temp = 0;
        double x_diff, y_diff;

        int x_div = 0, y_div = 0;

        if (calc_x < 300)
        {
            if (calc_y < 715)
            {
                x_div = 2;
                y_div = 6;
            }
            else
            {
                x_div = 2;
                y_div = 7;
            }
        }
        else if (calc_x > 600)
        {
            if (calc_y < 715)
            {
                x_div = 6;
                y_div = 6;
            }
            else
            {
                x_div = 6;
                y_div = 7;
            }
        }
        else
        {
            x_div = (int)(calc_x / 100);
            y_div = (int)((calc_y - 15) / 100);
            if (calc_y == 815) y_div = y_div - 1;
        }

        double slope = 0, k = 0;

        slope = (XYZ_Position[(y_div + 1) * 10 + x_div][1] - XYZ_Position[y_div * 10 + (x_div + 1)][1]) /
                (XYZ_Position[(y_div + 1) * 10 + x_div][0] - XYZ_Position[y_div * 10 + (x_div + 1)][0]);
        k = XYZ_Position[(y_div + 1) * 10 + x_div][1] - slope * XYZ_Position[(y_div + 1) * 10 + x_div][0];

        x_diff = calc_x - x_div * 100;
        y_diff = calc_y - y_div * 100;

    //*
    #if defined(Print_Detail)
        printf("X pos: %.5lf Y pos: %.5lf\n", calc_x, calc_y);
        printf("slope: %.5lf k: %.5lf\n", slope, k);
        printf("X/100: %.5lf y/100: %.5lf x_div: %d, y_div: %d\n", calc_x / 100, calc_y / 100, x_div, y_div);

        printf("a1: %.5lf b1: %.5lf c1: %.5lf d1: %.5lf\n", a_1[y_div * 10 + x_div], b_1[y_div * 10 + x_div], c_1[y_div * 10 + x_div],
               d_1[y_div * 10 + x_div]);
        printf("a2: %.5lf b2: %.5lf c2: %.5lf d2: %.5lf\n", a_2[y_div * 10 + x_div], b_2[y_div * 10 + x_div], c_2[y_div * 10 + x_div],
               d_2[y_div * 10 + x_div]);
        printf("z_pos1: %lf z_pos2: %lf\n",
               (-d_1[y_div * 10 + x_div] - a_1[y_div * 10 + x_div] * calc_x - b_1[y_div * 10 + x_div] * calc_y) / c_1[y_div * 10 + x_div],
               (-d_2[y_div * 10 + x_div] - a_2[y_div * 10 + x_div] * calc_x - b_2[y_div * 10 + x_div] * calc_y) / c_2[y_div * 10 + x_div]);
    #endif
        //*/
    #if defined(Print_Detail)
        printf("Z real: %.5lf\n", XYZ_Position[y_div * 10 + x_div][2]);
    #endif
            if (((-slope * calc_x + calc_y) <= k) || (a_2[y_div * 10 + x_div] == 0 && a_2[y_div * 10 + x_div] == 0 && b_2[y_div * 10 + x_div] == 0 &&
                                                  c_2[y_div * 10 + x_div] == 0 && d_2[y_div * 10 + x_div] == 0))
        {
            z_pos_temp = (-d_1[y_div * 10 + x_div] - a_1[y_div * 10 + x_div] * calc_x - b_1[y_div * 10 + x_div] * calc_y) / c_1[y_div * 10 + x_div];
        }
        else
        {
            z_pos_temp = (-d_2[y_div * 10 + x_div] - a_2[y_div * 10 + x_div] * calc_x - b_2[y_div * 10 + x_div] * calc_y) / c_2[y_div * 10 + x_div];
        }
        switch (choice)
        {
            case true:
                // Unit: mm -> m
                z_pos_temp = z_pos_temp / 1000;
                break;
            case false:
                z_pos_temp = z_pos_temp;
                break;
        }
    #if defined(Print_Detail)
        printf("%.5lf %.5lf %.5lf\n", calc_x, calc_y, z_pos_temp);
    #endif
            return z_pos_temp;
    }
    else
        return 9.99;
}
#endif

void Produce_Joint_XYZ(void)
{
    FILE *pFILE;
    pFILE = fopen("circle/Joint_XYZ.txt", "w");
    FILE *pFILE1;
    pFILE1 = fopen("circle/Joint_XYZ_Add.txt", "w");

    for (int i = 0; i < 100; i++)
    {
        for (double z_temp = 5.00; z_temp >= -5.00; z_temp = z_temp - 0.01)
        {
            if (find_region_in_out_d(XYZ_Position[i][0], XYZ_Position[i][1]))
                fprintf(pFILE, "G0 X%.5lf Y%.5lf Z%.2lf\n", XYZ_Position[i][0], XYZ_Position[i][1], z_temp);
            else
                fprintf(pFILE, ";G0 X%.5lf Y%.5lf Z%.2lf\n", XYZ_Position[i][0], XYZ_Position[i][1], z_temp);
        }
        // if (find_region_in_out_d(XYZ_Position[i][0], XYZ_Position[i][1]))
        //     fprintf(pFILE, "G0 X%.5lf Y%.5lf Z%.2lf\n", XYZ_Position[i][0], XYZ_Position[i][1], 20.0);
    }

    for (int i = 0; i < 13; i++)
    {
        for (double z_temp = 5.00; z_temp >= -5.00; z_temp = z_temp - 0.01)
        {
            fprintf(pFILE1, "G0 X%.5lf Y%.5lf Z%.2lf\n", XYZ_Position1_Add[i][0], XYZ_Position1_Add[i][1], z_temp);
        }
        // if (find_region_in_out_d(XYZ_Position[i][0], XYZ_Position[i][1]))
        // fprintf(pFILE1, "G0 X%.5lf Y%.5lf Z%.2lf\n", XYZ_Position1_Add[i][0], XYZ_Position1_Add[i][1], 20.0);
    }

    fclose(pFILE);
    fclose(pFILE1);
}

void XYZ_transformation_Joint_abc(void)
{
    /*
    FILE *pFILE;
    pFILE = fopen("circle/new Joint_XYZ.txt", "r");
    FILE *pFILE_Add;
    pFILE_Add = fopen("circle/new Joint_XYZ_Add.txt", "r");
    FILE *pFILE_5to4;
    pFILE_5to4 = fopen("circle/new Joint_XYZ_5to4.txt", "r");
    //*/

    FILE *pFILE;
    pFILE = fopen("circle/tra_Joint_XYZ.txt", "r");
    FILE *pFILE_Add;
    pFILE_Add = fopen("circle/tra_Joint_XYZ_Add.txt", "r");

    FILE *pFILE_Write;
    pFILE_Write = fopen("circle/Joint_XYZ_Z20_0_Neg20.txt", "w");
    FILE *pFILE_Add_Write;
    pFILE_Add_Write = fopen("circle/Joint_XYZ_Z20_0_Neg20_Add.txt", "w");

    FILE *pFILEa;
    pFILEa = fopen("circle/Joint_curvea.cpp", "w");
    FILE *pFILEb;
    pFILEb = fopen("circle/Joint_curveb.cpp", "w");
    FILE *pFILEc;
    pFILEc = fopen("circle/Joint_curvec.cpp", "w");

    FILE *pFILEa_txt;
    pFILEa_txt = fopen("circle/Joint_curvea.txt", "w");
    FILE *pFILEb_txt;
    pFILEb_txt = fopen("circle/Joint_curveb.txt", "w");
    FILE *pFILEc_txt;
    pFILEc_txt = fopen("circle/Joint_curvec.txt", "w");

    fprintf(pFILEa, "#include <avr/pgmspace.h>\n\n");
    fprintf(pFILEb, "#include <avr/pgmspace.h>\n\n");
    fprintf(pFILEc, "#include <avr/pgmspace.h>\n\n");

    double tmep_x, temp_y, temp_z;
    double temp_j, temp_a, temp_b, temp_c, temp_d;

    // double temp_calc_x, temp_calc_y, temp_calc_z[3];
    // double temp_calc_Joint[3][5];

    // int temp_gcode_num;

    int counter_point[3] = {0};

    char str1[3];
    //*
    for (int j = 0; j < 100; j++)
    {
        for (int i = 0; i < 1001; i++)
        {
            if (find_region_in_out_d(XYZ_Position[j][0], XYZ_Position[j][1]))
            {
                fscanf(pFILE, "%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, &temp_j, &temp_a, &temp_b, &temp_c, &temp_d, &tmep_x, &temp_y,
                       &temp_z);

                if (temp_z == 5.00 || temp_z == 0.00 || temp_z == -5.00)
                {
#if defined(Print_Detail)
                    printf("%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, temp_j, temp_a, temp_b, temp_c, temp_d, tmep_x, temp_y, temp_z);
#endif
                    fprintf(pFILE_Write, "%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, temp_j, temp_a, temp_b, temp_c, temp_d, tmep_x, temp_y,
                            temp_z);
                    counter_point[0]++;
                }
            }
            else
            {
                fscanf(pFILE, "%s X%lf Y%lf Z%lf\n", str1, &tmep_x, &temp_y, &temp_z);
                if (temp_z == 5.00 || temp_z == 0.00 || temp_z == -5.00)
                {
                    temp_j = 0;
                    temp_a = 0;
                    temp_b = 0;
                    temp_c = 0;
                    temp_d = 0;

#if defined(Print_Detail)
                    printf("%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, temp_j, temp_a, temp_b, temp_c, temp_d, tmep_x, temp_y, temp_z);
#endif
                    fprintf(pFILE_Write, "%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, temp_j, temp_a, temp_b, temp_c, temp_d, tmep_x, temp_y,
                            temp_z);
                    counter_point[0]++;
                }
            }
        }
    }
    //*/

    //*
    for (int j = 0; j < 13; j++)
    {
        for (int i = 0; i < 1001; i++)
        {
            fscanf(pFILE_Add, "G0 J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", &temp_j, &temp_a, &temp_b, &temp_c, &temp_d, &tmep_x, &temp_y, &temp_z);
            if (temp_z == 5.00 || temp_z == 0.00 || temp_z == -5.00)
            {
#if defined(Print_Detail)
                printf("G0 J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", temp_j, temp_a, temp_b, temp_c, temp_d, tmep_x, temp_y, temp_z);
#endif
                fprintf(pFILE_Add_Write, "G0 J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", temp_j, temp_a, temp_b, temp_c, temp_d, tmep_x, temp_y,
                        temp_z);
                counter_point[1]++;
            }
        }
    }

    fclose(pFILE_Write);
    fclose(pFILE_Add_Write);

    FILE *pFILE_Read;
    pFILE_Read = fopen("circle/Joint_XYZ_Z20_0_Neg20.txt", "r");
    FILE *pFILE_Add_Read;
    pFILE_Add_Read = fopen("circle/Joint_XYZ_Z20_0_Neg20_Add.txt", "r");

    printf("tra_Joint_XYZ.txt pass points: %d\n", counter_point[0]);
    printf("tra_Joint_XYZ_Add.txt pass points: %d\n", counter_point[1]);

    // pFILE_Read,pFILE_Add_Read

    for (int j = 0; j < 100; j++)
    {
        clear_data();
        // Joint=aZ^2+bZ+c Joint=a(4000000)+b(+-2000)+c
        for (int i = 0; i < 3; i++)
        {
            fscanf(pFILE_Read, "%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, &temp_calc_Joint[i][0], &temp_calc_Joint[i][1],
                   &temp_calc_Joint[i][2], &temp_calc_Joint[i][3], &temp_calc_Joint[i][4], &temp_calc_x, &temp_calc_y, &temp_calc_z[i]);
#if defined(Print_Detail)
            printf("%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, temp_calc_Joint[i][0], temp_calc_Joint[i][1], temp_calc_Joint[i][2],
                   temp_calc_Joint[i][3], temp_calc_Joint[i][4], temp_calc_x, temp_calc_y, temp_calc_z[i]);
#endif
        }
#if defined(Print_Detail)
        printf("%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, temp_calc_Joint[0][0], temp_calc_Joint[0][1], temp_calc_Joint[0][2],
               temp_calc_Joint[0][3], temp_calc_Joint[0][4], temp_calc_x, temp_calc_y, temp_calc_z[0]);
#endif

        temp_calc_z[0] = temp_calc_z[0] * 100; //以0.01mm為單位
        temp_calc_z[1] = temp_calc_z[1] * 100; //以0.01mm為單位
        temp_calc_z[2] = temp_calc_z[2] * 100; //以0.01mm為單位

        int x_div = (int)(temp_calc_x / 100);
        int y_div = (int)(temp_calc_y / 100);
        int total = y_div * 10 + x_div;

        printf("X:%3.5lf Y:%3.5lf Total =", temp_calc_x, temp_calc_y);

        if (temp_calc_Joint[0][0] == 0 && temp_calc_Joint[0][1] == 0 && temp_calc_Joint[0][2] == 0 && temp_calc_Joint[0][3] == 0 &&
            temp_calc_Joint[0][4] == 0)
        {
            printf(" %d X\n", j);
            for (int ii = 0; ii < 5; ii++)
            {
                Jc[j][ii] = 0;
                Jb[j][ii] = 0;
                Ja[j][ii] = 0;
            }
        }
        else
        {
            printf(" %d V\n", j);
            for (int ii = 0; ii < 5; ii++)
            {
                Jc[j][ii] = temp_calc_Joint[1][ii];
                Jb[j][ii] = (temp_calc_Joint[0][ii] - temp_calc_Joint[2][ii]) / (temp_calc_z[0] * 2);
                Ja[j][ii] = (temp_calc_Joint[0][ii] - 500 * Jb[j][ii] - Jc[j][ii]) / (temp_calc_z[0] * temp_calc_z[0]);
            }
        }

#if defined(Print_Detail)
        printf("\nJa[0]:%.10lf Ja[1]:%.10lf Ja[2]:%.10lf Ja[3]:%.10lf Ja[4]:%.10lf\n", Ja[total][0], Ja[total][1], Ja[total][2], Ja[total][3],
               Ja[total][4]);
        printf("Jb[0]:%.10lf Jb[1]:%.10lf Jb[2]:%.10lf Jb[3]:%.10lf Jb[4]:%.10lf\n", Jb[total][0], Jb[total][1], Jb[total][2], Jb[total][3],
               Jb[total][4]);
        printf("Jc[0]:%.10lf Jc[1]:%.10lf Jc[2]:%.10lf Jc[3]:%.10lf Jc[4]:%.10lf\n\n", Jc[total][0], Jc[total][1], Jc[total][2], Jc[total][3],
               Jc[total][4]);
#endif
    }

    for (int i = 0; i < 100; i++)
    {
        // .cpp
        if (i == 0)
            fprintf(pFILEa, "const PROGMEM float a_m2[500] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Ja[i][0], Ja[i][1], Ja[i][2], Ja[i][3],
                    Ja[i][4]);
        else if (i == 99)
            fprintf(pFILEa, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Ja[i][0], Ja[i][1], Ja[i][2], Ja[i][3], Ja[i][4]);
        else
            fprintf(pFILEa, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Ja[i][0], Ja[i][1], Ja[i][2], Ja[i][3], Ja[i][4]);

        if (i == 0)
            fprintf(pFILEb, "const PROGMEM float b_m2[500] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jb[i][0], Jb[i][1], Jb[i][2], Jb[i][3],
                    Jb[i][4]);
        else if (i == 99)
            fprintf(pFILEb, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Jb[i][0], Jb[i][1], Jb[i][2], Jb[i][3], Jb[i][4]);
        else
            fprintf(pFILEb, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jb[i][0], Jb[i][1], Jb[i][2], Jb[i][3], Jb[i][4]);
        if (i == 0)
            fprintf(pFILEc, "const PROGMEM float c_m2[500] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jc[i][0], Jc[i][1], Jc[i][2], Jc[i][3],
                    Jc[i][4]);
        else if (i == 99)
            fprintf(pFILEc, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Jc[i][0], Jc[i][1], Jc[i][2], Jc[i][3], Jc[i][4]);
        else
            fprintf(pFILEc, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jc[i][0], Jc[i][1], Jc[i][2], Jc[i][3], Jc[i][4]);

        // txt
        if (i == 0)
            fprintf(pFILEa_txt, "const PROGMEM float a_m2[500] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Ja[i][0], Ja[i][1], Ja[i][2],
                    Ja[i][3], Ja[i][4]);
        else if (i == 99)
            fprintf(pFILEa_txt, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Ja[i][0], Ja[i][1], Ja[i][2], Ja[i][3], Ja[i][4]);
        else
            fprintf(pFILEa_txt, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Ja[i][0], Ja[i][1], Ja[i][2], Ja[i][3], Ja[i][4]);

        if (i == 0)
            fprintf(pFILEb_txt, "const PROGMEM float b_m2[500] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jb[i][0], Jb[i][1], Jb[i][2],
                    Jb[i][3], Jb[i][4]);
        else if (i == 99)
            fprintf(pFILEb_txt, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Jb[i][0], Jb[i][1], Jb[i][2], Jb[i][3], Jb[i][4]);
        else
            fprintf(pFILEb_txt, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jb[i][0], Jb[i][1], Jb[i][2], Jb[i][3], Jb[i][4]);

        if (i == 0)
            fprintf(pFILEc_txt, "const PROGMEM float c_m2[500] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jc[i][0], Jc[i][1], Jc[i][2],
                    Jc[i][3], Jc[i][4]);
        else if (i == 99)
            fprintf(pFILEc_txt, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Jc[i][0], Jc[i][1], Jc[i][2], Jc[i][3], Jc[i][4]);
        else
            fprintf(pFILEc_txt, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jc[i][0], Jc[i][1], Jc[i][2], Jc[i][3], Jc[i][4]);
    }
    //*/
    for (int j = 0; j < 13; j++)
    {
        // Joint=aZ^2+bZ+c Joint=a(4000000)+b(+-2000)+c
        for (int i = 0; i < 3; i++)
        {
            fscanf(pFILE_Add_Read, "G0 J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", &temp_calc_Joint[i][0], &temp_calc_Joint[i][1],
                   &temp_calc_Joint[i][2], &temp_calc_Joint[i][3], &temp_calc_Joint[i][4], &temp_calc_x, &temp_calc_y, &temp_calc_z[i]);
            // #if defined(Print_Detail)
            printf("%s J%lf A%lf B%lf C%lf D%lf X%lf Y%lf Z%lf\n", str1, temp_calc_Joint[i][0], temp_calc_Joint[i][1], temp_calc_Joint[i][2],
                   temp_calc_Joint[i][3], temp_calc_Joint[i][4], temp_calc_x, temp_calc_y, temp_calc_z[i]);
            // #endif
        }

        temp_calc_z[0] = temp_calc_z[0] * 100; //以0.01mm為單位
        temp_calc_z[1] = temp_calc_z[1] * 100; //以0.01mm為單位
        temp_calc_z[2] = temp_calc_z[2] * 100; //以0.01mm為單位

        int x_div = (int)(temp_calc_x / 100);
        int y_div = (int)(temp_calc_y / 100);
        int total = y_div * 10 + x_div;

        for (int ii = 0; ii < 5; ii++)
        {
            Jc_Add[j][ii] = temp_calc_Joint[1][ii];
            Jb_Add[j][ii] = (temp_calc_Joint[0][ii] - temp_calc_Joint[2][ii]) / (temp_calc_z[0] * 2);
            Ja_Add[j][ii] = (temp_calc_Joint[0][ii] - 500 * Jb[j][ii] - Jc[j][ii]) / (temp_calc_z[0] * temp_calc_z[0]);
        }
#if defined(Print_Detail)
        printf("\nJa_Add[0]:%.10lf Ja_Add[1]:%.10lf Ja_Add[2]:%.10lf Ja_Add[3]:%.10lf Ja_Add[4]:%.10lf\n", Ja_Add[j][0], Ja_Add[j][1], Ja_Add[j][2],
               Ja_Add[j][3], Ja_Add[j][4]);
        printf("Jb_Add[0]:%.10lf Jb_Add[1]:%.10lf Jb_Add[2]:%.10lf Jb_Add[3]:%.10lf Jb_Add[4]:%.10lf\n", Jb_Add[j][0], Jb_Add[j][1], Jb_Add[j][2],
               Jb_Add[j][3], Jb_Add[j][4]);
        printf("Jc_Add[0]:%.10lf Jc_Add[1]:%.10lf Jc_Add[2]:%.10lf Jc_Add[3]:%.10lf Jc_Add[4]:%.10lf\n\n", Jc_Add[j][0], Jc_Add[j][1], Jc_Add[j][2],
               Jc_Add[j][3], Jc_Add[j][4]);
#endif
    }
    //*
    for (int i = 0; i < 13; i++)
    {
        if (i == 0)
            fprintf(pFILEa, "const PROGMEM float a_m2_Add[65] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Ja_Add[i][0], Ja_Add[i][1],
                    Ja_Add[i][2], Ja_Add[i][3], Ja_Add[i][4]);
        else if (i == 12)
            fprintf(pFILEa, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Ja_Add[i][0], Ja_Add[i][1], Ja_Add[i][2], Ja_Add[i][3], Ja_Add[i][4]);
        else
            fprintf(pFILEa, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Ja_Add[i][0], Ja_Add[i][1], Ja_Add[i][2], Ja_Add[i][3], Ja_Add[i][4]);

        if (i == 0)
            fprintf(pFILEb, "const PROGMEM float b_m2_Add[65] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jb_Add[i][0], Jb_Add[i][1],
                    Jb_Add[i][2], Jb_Add[i][3], Jb_Add[i][4]);
        else if (i == 12)
            fprintf(pFILEb, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Jb_Add[i][0], Jb_Add[i][1], Jb_Add[i][2], Jb_Add[i][3], Jb_Add[i][4]);
        else
            fprintf(pFILEb, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jb_Add[i][0], Jb_Add[i][1], Jb_Add[i][2], Jb_Add[i][3], Jb_Add[i][4]);

        if (i == 0)
            fprintf(pFILEc, "const PROGMEM float c_m2_Add[65] = {\n\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jc_Add[i][0], Jc_Add[i][1],
                    Jc_Add[i][2], Jc_Add[i][3], Jc_Add[i][4]);
        else if (i == 12)
            fprintf(pFILEc, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf\n};\n", Jc_Add[i][0], Jc_Add[i][1], Jc_Add[i][2], Jc_Add[i][3], Jc_Add[i][4]);
        else
            fprintf(pFILEc, "\t%.10lf, %.10lf, %.10lf, %.10lf, %.10lf,\n", Jc_Add[i][0], Jc_Add[i][1], Jc_Add[i][2], Jc_Add[i][3], Jc_Add[i][4]);
    }
    //*/

    /*
    for (int i = 0; i < 14; i++)
    {
        if (i == 0)
            fprintf(pFILEa, "a_m1_5to4[100][5]=\n{{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf},\n", Ja[i][0], Ja[i][1], Ja[i][2], Ja[i][3], Ja[i][4]);
        else if (i == 13)
            fprintf(pFILEa, "{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf}};\n", Ja[i][0], Ja[i][1], Ja[i][2], Ja[i][3], Ja[i][4]);
        else
            fprintf(pFILEa, "{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf},\n", Ja[i][0], Ja[i][1], Ja[i][2], Ja[i][3], Ja[i][4]);

        if (i == 0)
            fprintf(pFILEb, "b_m1_5to4[100][5]=\n{{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf},\n", Jb[i][0], Jb[i][1], Jb[i][2], Jb[i][3], Jb[i][4]);
        else if (i == 13)
            fprintf(pFILEb, "{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf}};\n", Jb[i][0], Jb[i][1], Jb[i][2], Jb[i][3], Jb[i][4]);
        else
            fprintf(pFILEb, "{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf},\n", Jb[i][0], Jb[i][1], Jb[i][2], Jb[i][3], Jb[i][4]);

        if (i == 0)
            fprintf(pFILEc, "c_m1_5to4[100][5]=\n{{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf},\n", Jc[i][0], Jc[i][1], Jc[i][2], Jc[i][3], Jc[i][4]);
        else if (i == 13)
            fprintf(pFILEc, "{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf}};\n", Jc[i][0], Jc[i][1], Jc[i][2], Jc[i][3], Jc[i][4]);
        else
            fprintf(pFILEc, "{%.10lf, %.10lf, %.10lf, %.10lf, %.10lf},\n", Jc[i][0], Jc[i][1], Jc[i][2], Jc[i][3], Jc[i][4]);
    }
    //*/

    fclose(pFILE);
    fclose(pFILE_Add);
    fclose(pFILE_Read);
    fclose(pFILE_Add_Read);
    fclose(pFILEa);
    fclose(pFILEb);
    fclose(pFILEc);
    fclose(pFILEa_txt);
    fclose(pFILEb_txt);
    fclose(pFILEc_txt);

    /*printf("Ja[J]:\n%d    %d      %d      %d      %d      %d      %d      %d      %d      %d\n", 0, 100, 200, 300, 400, 500, 600, 700, 800,
    870); for (int i = 0; i < 10; i++)
    {
        printf("%.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf %.10lf\n", Ja[i * 10 + 0][0], Ja[i * 10 + 1][0], Ja[i * 10 + 2][0],
               Ja[i * 10 + 3][0], Ja[i * 10 + 4][0], Ja[i * 10 + 5][0], Ja[i * 10 + 6][0], Ja[i * 10 + 7][0], Ja[i * 10 + 8][0], Ja[i * 10 +
    9][0]);
    }*/
    // double Ja[100][5], Jb[100], Jc[100][5];
    // double Ja_Add[100][5], Jb_Add[100][5], Jc_Add[100][5];
    // double Ja_5to4[100][5], Jb_5to4[100][5], Jc_5to4[100][5];

    printf("\n");
    printf("Copy Joint_curvea.cpp,Joint_curveb.cpp and Joint_curvec.cpp ...\n");
    system("powershell copy 'circle/Joint_curvea.cpp' 'D:/Users/admin/Desktop/Marlin/20191127/Marlin'");
    system("powershell copy 'circle/Joint_curveb.cpp' 'D:/Users/admin/Desktop/Marlin/20191127/Marlin'");
    system("powershell copy 'circle/Joint_curvec.cpp' 'D:/Users/admin/Desktop/Marlin/20191127/Marlin'");
    printf("\n");
    printf("Copy completed !!");
}

void clear_data(void)
{
    for (int i1 = 0; i1 < 3; i1++)
    {
        for (int ii = 0; ii < 5; ii++)
        {
            temp_calc_Joint[i1][ii] = 0;
        }
        temp_calc_z[i1] = 0;
    }
    temp_calc_x = 0;
    temp_calc_y = 0;
}

int Use_XY_to_Matrix_Index(double UXYMIX, double UXYMIY)
{
#if defined(Print_Detail)
    printf(">>> Use_XY_to_Matrix_Index");
#endif
    int temp_UXYMIX = (int)UXYMIX;
    int temp_UXYMIY = (int)UXYMIY;
    int temp_return = 0;

    switch (temp_UXYMIX)
    {
        case 209:
            switch (temp_UXYMIY)
            {
                case 615:
                    temp_return = 62;
                    break;
                case 715:
                    temp_return = 72;
                    break;
                case 815:
                    temp_return = 82;
                    break;
            }
            break;
        case 300:
            switch (temp_UXYMIY)
            {
                case 615:
                    temp_return = 63;
                    break;
                case 715:
                    temp_return = 73;
                    break;
                case 815:
                    temp_return = 83;
                    break;
            }
            break;
        case 400:
            switch (temp_UXYMIY)
            {
                case 615:
                    temp_return = 64;
                    break;
                case 715:
                    temp_return = 74;
                    break;
                case 815:
                    temp_return = 84;
                    break;
            }
            break;
        case 500:
            switch (temp_UXYMIY)
            {
                case 615:
                    temp_return = 65;
                    break;
                case 715:
                    temp_return = 75;
                    break;
                case 815:
                    temp_return = 85;
                    break;
            }
            break;
        case 600:
            switch (temp_UXYMIY)
            {
                case 615:
                    temp_return = 66;
                    break;
                case 715:
                    temp_return = 76;
                    break;
                case 815:
                    temp_return = 86;
                    break;
            }
            break;
        case 670:
            switch (temp_UXYMIY)
            {
                case 615:
                    temp_return = 67;
                    break;
                case 715:
                    temp_return = 77;
                    break;
                case 815:
                    temp_return = 87;
                    break;
            }
            break;
    }
#if defined(Print_Detail)
    printf("Return Value: %d", temp_return);
    printf("<<< Use_XY_to_Matrix_Index");
#endif
    return temp_return;
}