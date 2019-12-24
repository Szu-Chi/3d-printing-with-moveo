#include "Mesh.h"

int main(void)
{
  Load_Mesh();
  Print_Mesh(); // can Comment
  calc_abcd();

  printf("\nPlease enter X position:");
  scanf("%lf", &x_pos);
  printf("Please enter Y position:");
  scanf("%lf", &y_pos);
  printf("You enter X:%.2lf Y:%.2lf\n", x_pos, y_pos);

  z_pos = calc_z(x_pos, y_pos);
  printf("Your z:%.10f", z_pos);

  // system("pause");
  return 0;
}