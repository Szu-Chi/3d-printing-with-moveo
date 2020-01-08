#include <stdio.h>
#include <stdlib.h>
#include <ros/package.h>

#define XYZ 3

double a_1[4][4], b_1[4][4], c_1[4][4], d_1[4][4];
double a_2[4][4], b_2[4][4], c_2[4][4], d_2[4][4];
double differ_ab_1[4][4], differ_ac_1[4][4];
double differ_ab_2[4][4], differ_ac_2[4][4];
double x_pos, y_pos, z_pos;

void Print_Mesh(void);
void calc_abcd(void);
void Print_data(const double Pr_buffer[4][4]);
void Load_Mesh(void);
double calc_z(double calc_x, double calc_y);

/*double XYZ_Position[25][XYZ] = {
    {0, 0, 0.01},   {45, 0, 0.02},   {90, 0, 0.02},   {135, 0, 0.01},   {180, 0, 0.02},
    {0, 45, 0.02},  {45, 45, 0.05},  {90, 45, 0.03},  {135, 45, 0.02},  {180, 45, 0.01},
    {0, 90, 0.03},  {45, 90, 0.05},  {90, 90, 0.04},  {135, 90, 0.03},  {180, 90, 0.03},
    {0, 135, 0.03}, {45, 135, 0.01}, {90, 135, 0.01}, {135, 135, 0.03}, {180, 135, 0.04},
    {0, 180, 0.05}, {45, 180, 0.02}, {90, 180, 0.01}, {135, 180, 0.04}, {180, 180, 0.05}};*/

double XYZ_Position[25][XYZ] = {{0, 0, 0},    {45, 0, 0},   {90, 0, 0},  {135, 0, 0},  {180, 0, 0},  {0, 45, 0},    {45, 45, 0},   {90, 45, 0}, {135, 45, 0}, {180, 45, 0}, {0, 90, 0},    {45, 90, 0},  {90, 90, 0},
                                {135, 90, 0}, {180, 90, 0}, {0, 135, 0}, {45, 135, 0}, {90, 135, 0}, {135, 135, 0}, {180, 135, 0}, {0, 180, 0}, {45, 180, 0}, {90, 180, 0}, {135, 180, 0}, {180, 180, 0}};

void Print_Mesh(void)
{
  //std::printf("+------------------------------------------------------+\n");
  //std::printf("|          |                   X                       |\n");
  //std::printf("+    Z     +-------------------------------------------+\n");
  //std::printf("|          | %+3d       %+3d      %+3d      %+3d     %+3d |\n", 0, 45, 90, 135, 180);
  //std::printf("+---+------+-------------------------------------------+\n");
  //std::printf("|   | %+3d  |  %+.2lf    %+.2lf    %+.2lf    %+.2lf    %+.2lf|\n", 0, XYZ_Position[0][2], XYZ_Position[1][2], XYZ_Position[2][2], XYZ_Position[3][2], XYZ_Position[4][2]);
  //std::printf("|   | %+3d  |  %+.2lf    %+.2lf    %+.2lf    %+.2lf    %+.2lf|\n", 45, XYZ_Position[5][2], XYZ_Position[6][2], XYZ_Position[7][2], XYZ_Position[8][2], XYZ_Position[9][2]);
  //std::printf("| Y | %+3d  |  %+.2lf    %+.2lf    %+.2lf    %+.2lf    %+.2lf|\n", 90, XYZ_Position[10][2], XYZ_Position[11][2], XYZ_Position[12][2], XYZ_Position[13][2], XYZ_Position[14][2]);
  //std::printf("|   | %+3d |  %+.2lf    %+.2lf    %+.2lf    %+.2lf    %+.2lf|\n", 135, XYZ_Position[15][2], XYZ_Position[16][2], XYZ_Position[17][2], XYZ_Position[18][2], XYZ_Position[19][2]);
  //std::printf("|   | %+3d |  %+.2lf    %+.2lf    %+.2lf    %+.2lf    %+.2lf|\n", 180, XYZ_Position[20][2], XYZ_Position[21][2], XYZ_Position[22][2], XYZ_Position[23][2], XYZ_Position[24][2]);
  //std::printf("+---+------+-------------------------------------------+\n");
}

void calc_abcd(void)
{
  for (int j = 0; j < 4; j++)
  {
    for (int i = 0; i < 4; i++)
    {
      // AB
      differ_ab_1[j][i] = XYZ_Position[j * 5 + (i + 1)][2] - XYZ_Position[j * 5 + i][2];
      differ_ab_2[j][i] = XYZ_Position[j * 5 + (i + 5)][2] - XYZ_Position[j * 5 + (i + 5 + 1)][2];
      // AC
      differ_ac_1[j][i] = XYZ_Position[j * 5 + (i + 5)][2] - XYZ_Position[j * 5 + i][2];
      differ_ac_2[j][i] = XYZ_Position[j * 5 + (i + 1)][2] - XYZ_Position[j * 5 + ((i + 1) + 5)][2];
    }
  }

  for (int j = 0; j < 4; j++)
  {
    for (int i = 0; i < 4; i++)
    {
      a_1[j][i] = 0 * differ_ac_1[j][i] - 45 * differ_ab_1[j][i];
      b_1[j][i] = -45 * differ_ac_1[j][i] + 0 * differ_ab_1[j][i];
      c_1[j][i] = 45 * 45;

      a_2[j][i] = -0 * differ_ac_2[j][i] + 45 * differ_ab_2[j][i];
      b_2[j][i] = 45 * differ_ac_2[j][i] - 0 * differ_ab_2[j][i];
      c_2[j][i] = 45 * 45;
    }
  }

  for (int j = 0; j < 4; j++)
  {
    for (int i = 0; i < 4; i++)
    {
      d_1[j][i] = -a_1[j][i] * XYZ_Position[j * 5 + i][0] - b_1[j][i] * XYZ_Position[j * 5 + i][1] - c_1[j][i] * XYZ_Position[j * 5 + i][2];
      d_2[j][i] = -a_2[j][i] * XYZ_Position[j * 5 + (i + 6)][0] - b_2[j][i] * XYZ_Position[j * 5 + (i + 6)][1] - c_2[j][i] * XYZ_Position[j * 5 + (i + 6)][2];
    }
  }

  //*
  Print_data(differ_ab_1);
  Print_data(differ_ac_1);
  Print_data(differ_ab_2);
  Print_data(differ_ac_2);

  Print_data(a_1);
  Print_data(b_1);
  Print_data(c_1);

  Print_data(a_2);
  Print_data(b_2);
  Print_data(c_2);

  Print_data(d_1);
  Print_data(d_2);
  //*/
}

void Print_data(const double Pr_buffer[4][4])
{
  for (int j = 0; j < 4; j++)
  {
    for (int i = 0; i < 4; i++)
    {
      //std::printf("    %.2lf    ", Pr_buffer[j][i]);
    }
    //std::printf("\n");
  }
  //std::printf("\n");
}

double calc_z(double calc_x, double calc_y)
{
  double z_pos_temp = 0;
  double x_diff, y_diff;
  int x_div = (int)((calc_x - 1) / 45);
  int y_div = (int)((calc_y - 1) / 45);
  if (x_div < 0)
    x_div = 0;
  if (y_div < 0)
    y_div = 0;
  //std::printf("x_div=%d y_div=%d\n", x_div, y_div);
  //std::printf("a_1=%+.10lf a_2=%+.10lf\n", a_1[y_div][x_div], a_2[y_div][x_div]);
  //std::printf("b_1=%+.10lf b_2=%+.10lf\n", b_1[y_div][x_div], b_2[y_div][x_div]);
  //std::printf("c_1=%+4.10lf c_2=%+4.10lf\n", c_1[y_div][x_div], c_2[y_div][x_div]);
  //std::printf("d_1=%+.10lf d_2=%+.10lf\n", d_1[y_div][x_div], d_2[y_div][x_div]);

  x_diff = calc_x - x_div * 45;
  y_diff = calc_y - y_div * 45;

  //std::printf("x_diff:%.3lf y_diff:%.3lf\n", x_diff, y_diff);

  if ((x_diff + y_diff) <= 45)
  {
    z_pos_temp = (-d_1[y_div][x_div] - a_1[y_div][x_div] * calc_x - b_1[y_div][x_div] * calc_y) / c_1[y_div][x_div];
    //std::printf("a_1*x_pos=%.10lf b_1*y_pos=%.10lf \n", a_1[y_div][x_div] * calc_x, b_1[y_div][x_div] * calc_y);
    //std::printf("b_1*x_pos=%.10lf d_1=%.10lf \n", c_1[y_div][x_div] * z_pos_temp, c_1[y_div][x_div]);
  }
  else
  {
    z_pos_temp = (-d_2[y_div][x_div] - a_2[y_div][x_div] * calc_x - b_2[y_div][x_div] * calc_y) / c_1[y_div][x_div];
    //std::printf("a_2*x_pos=%.10lf b_2*y_pos=%.10lf \n", a_2[y_div][x_div] * calc_x, b_2[y_div][x_div] * calc_y);
    //std::printf("c_2*z_pos=%.10lf d_2=%.10lf \n", c_2[y_div][x_div] * z_pos_temp, d_2[y_div][x_div]);
  }
  return z_pos_temp;
}

void Load_Mesh(void)
{
  FILE *ReadFILE;
  std::string path = ros::package::getPath("gcode_translation");
  path += "/include/Mesh/Mesh.txt";
  ReadFILE = fopen(path.c_str(), "r");

  for (int i = 0; i < 5; i++)
    fscanf(ReadFILE, "%lf %lf %lf %lf %lf\n", &XYZ_Position[5 * i + 0][2], &XYZ_Position[5 * i + 1][2], &XYZ_Position[5 * i + 2][2], &XYZ_Position[5 * i + 3][2], &XYZ_Position[5 * i + 4][2]);

  fclose(ReadFILE);
}