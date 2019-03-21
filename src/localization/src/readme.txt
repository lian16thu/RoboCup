"f_LUT_x.txt" and "f_LUT_y.txt" store f_x and f_y from pixel [0][179] to pixel [479][359]
f_x and f_y of other pixels are all 0
you can read them as the following code shows

//**********************************************************************
int f_LUT_x[480][360];
int f_LUT_y[480][360];
ifstream fin_x;
ifstream fin_y;
fin_x.open("/home/hqq1/catkin_ws/f_LUT_x.txt");
fin_y.open("/home/hqq1/catkin_ws/f_LUT_y.txt");
for(int i = 0; i < 179; i++)
     {
       for(int j = 0; j < 480; j++)
        {
           f_LUT_x[j][i] = 0;
           f_LUT_y[j][i] = 0;
        }
      }
for(int i = 179; i < 360; i++)
     {
       for(int j = 0; j < 480; j++)
        {
           fin_x >> f_LUT_x[j][i];
           fin_y >> f_LUT_y[j][i];
        }
      }

