#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

double distance_between_two_points_calculating(double x1, double y1, double x2, double y2)
{
    double a = pow((x2 - x1), (double)2.0) + pow((y2 - y1), (double)2.0);
    double b = sqrt(a);
    return b;
}

double arithmetic_mean(double v1, double v2)
{
    double a = v1 + v2;
    double b = 2.0;
    double rtn = a / b;
    return rtn;
}

double* rotation_matrix(double x, double y, double ax, double ay, double angle)
{
    double* rtn;
    double xy[2] = {
        x * cos(angle) - y * sin(angle) + ax,
        x * sin(angle) + y * cos(angle) + ay
    };
    rtn = xy;
    return rtn;
}

double getRadians(double degree)
{
    double pi = 3.1415926535897;
    double d = 180.0;
    return degree * (pi / d);
}

double getDegrees(double radian)
{
    double pi = 3.1415926535897;
    double d = 180.0;
    return radian * (d / pi);
}

int point_location_match(int x, int y, int* list_x, int* list_y, int length)
{
    int cnt = 0;
    for (cnt = 0; cnt < length; cnt++)
    {
        if ((list_x[cnt] == x) && (list_y[cnt] == y))
            return 1;
    }
    return 0;
}

int xycar_min(int num1, int num2)
{
    if(num1 < num2)
    {
        return num1;
    }
    else if(num1 > num2)
    {
        return num2;
    }
    else
    {
        return num1;
    }
}

double min_3d(double num1, double num2, double num3)
{
    double minv = 0.0;
    if (num1 <= num2) minv = num1;
    else return minv = num2;

    if (minv > num3) minv = num3;
    return minv;
}

double min_d(double num1, double num2)
{
    if (num1 < num2) return num1;
    else if (num1 > num2) return num2;
    else return num1;
}

int xycar_max(int num1, int num2)
{
    if (num1 < num2) return num2;
    else if (num1 > num2) return num1;
    else return num1;
}

double max_d(double num1, double num2)
{
    if (num1 < num2) return num2;
    else if (num1 > num2) return num1;
    else return num1;
}

int point_surface_match(int x, int y, int* list_x, int* list_y, int length)
{
    int cnt = 0;
    int ccnt = 0;
    int cccnt = 0;
    int minv = 0;
    int maxv = 0;
    int* L_x;
    int* L_y;
    int tmp = 0;

    for (cnt = 0; cnt < length; cnt++)
    {
        for (ccnt = cnt + 1; ccnt < length; ccnt++)
        {
            if (list_y[cnt] == list_y[ccnt])
            {
                minv = xycar_min(list_x[cnt], list_x[ccnt]);
                maxv = xycar_max(list_x[cnt], list_x[ccnt]);
            }
        }
        tmp = maxv - minv - 1;
        L_x = (int*)malloc(sizeof(int) * tmp);
        L_y = (int*)malloc(sizeof(int) * tmp);
        for (cccnt = 0; cccnt < (maxv - minv - 1); cccnt++)
        {
            L_x[cccnt] = minv + cccnt;
            L_y[cccnt] = list_y[cnt];
        }
        if (point_location_match(x, y, L_x, L_y, (maxv - minv - 1)) == 1)
        {
            free(L_x);
            free(L_y);
            return 1;
        }
        free(L_x);
        free(L_y);
    }
    return 0;
}

double* points_output(int x, int y, double yaw, double angle, double detect_angle, int* obstract_x, int* obstract_y, int ob_length, int car_x, int car_y, int width, int height)
{
    double* rtn;

    int cnt = 0;
    int ccnt = 0;

    double X = 0.0;
    double Y = 0.0;

    int xidx_i = 0;
    int yidx_i = 0;
    double xidx_d = 0.0;
    double yidx_d = 0.0;

    double m = 0.0;
    double b = 0.0;

    double negative_radian_angle = 0.0;
    double degrees = 0.0;

    double zero = 0.0;
    double p90 = 90.0;
    double n90 = 270.0;
    double dx = (double)x;
    double dy = (double)y;
    double dwidth = (double)width;
    double dheight = (double)height;
    double dcarx = (double)car_x;
    double dcary = (double)car_y;

    double distance_calc = 0.0;

    int drive[2] = { 1, 1 };
    int buho[2] = { 1, -1 };
    double first_dist[2] = { 0.0, 0.0 };
    int return_array[4] = { -2, -2, -2, -2 };
    double final[3] = { -1.0, -1.0, -1.0 };

    int convolution_x[3] = { 0 };
    int convolution_y[3] = { 0 };

    int conv_cnt_1 = 0;
    int conv_cnt_2 = 0;
    int out_switch = 0;

    degrees = (-1) * getDegrees(getRadians(-(yaw + angle + detect_angle)));

    if (degrees < 0) degrees = fmod(degrees, 360.0) + 360.0;
    if (degrees >= 0) degrees = fmod(degrees, 360.0);

    negative_radian_angle = getRadians(-degrees);

    m = tan(negative_radian_angle);
    b = dy - (m * dx);

    for (cnt = 0; cnt < width; cnt++)
    {
        for (ccnt = 0; ccnt < 2; ccnt++)
        {
            xidx_i = x + buho[ccnt] * cnt;
            xidx_d = (double)xidx_i;

            yidx_i = y + buho[ccnt] * cnt;
            yidx_d = (double)yidx_i;

            if ((return_array[ccnt * 2] != -2) || (drive[ccnt] != 1)) continue;

            if ((0 <= xidx_i) && (xidx_i < width))
            {
                if ((degrees == p90) || (degrees == n90))
                {
                    X = dx;
                    Y = yidx_d;
                }
                else
                {
                    X = xidx_d;
                    Y = (m * X) + b;
                }

                distance_calc = distance_between_two_points_calculating(dcarx, dcary, X, Y);
                if (first_dist[ccnt] == 0.0) first_dist[ccnt] = distance_calc;
                if ((first_dist[ccnt] - distance_calc) > 0) drive[ccnt] = 0;

                if (drive[ccnt] != 1) continue;

                if ((zero <= Y) && (Y < dheight))
                {
                    if (point_location_match(X, Y, obstract_x, obstract_y, ob_length) == 1)
                    {
                        return_array[ccnt * 2] = (int)round(X);
                        return_array[ccnt * 2 + 1] = (int)round(Y);
                    }
                    //convolution_x[1] = (int)floor(X);
                    //convolution_x[0] = convolution_x[1] - 1;
                    //convolution_x[2] = convolution_x[1] + 1;

                    //convolution_y[1] = (int)floor(Y);
                    //convolution_y[0] = convolution_y[1] - 1;
                    //convolution_y[2] = convolution_y[1] + 1;

                    //for (conv_cnt_1 = 0; conv_cnt_1 < 3; conv_cnt_1++)
                    //{
                    //    for (conv_cnt_2 = 0; conv_cnt_2 < 3; conv_cnt_2++)
                    //    {
                    //        if (point_location_match(convolution_x[conv_cnt_1], convolution_y[conv_cnt_2], obstract_x, obstract_y, ob_length) == 1)
                    //        {
                    //            return_array[ccnt * 2] = convolution_x[conv_cnt_1];
                    //            return_array[ccnt * 2 + 1] = convolution_y[conv_cnt_2];
                    //            out_switch = 1;
                    //            break;
                    //        }
                    //    }
                    //    if (out_switch == 1) break;
                    //}
                }
                else
                {
                    if (zero > Y) Y = 0;
                    else Y = dheight;

                    if ((degrees == p90) || (degrees == n90)) X = dx;
                    else X = (Y - b) / m;

                    return_array[ccnt * 2] = X;
                    return_array[ccnt * 2 + 1] = Y;
                }
            }
            else if (xidx_i > dwidth)
            {
                return_array[0] = width;
                return_array[1] = (int)round((m * dwidth) + b);
                drive[ccnt] = 0;
            }
            else if (xidx_i < 0)
            {
                return_array[2] = 0;
                return_array[3] = (int)round(b);
                drive[ccnt] = 0;
            }
            if ((return_array[0] != -2) || (return_array[2] != -2)) break;
        }
    }

    if (return_array[0] == -2)
    {
        final[0] = (double)return_array[2];
        final[1] = (double)return_array[3];
    }
    else if (return_array[2] == -2)
    {
        final[0] = (double)return_array[0];
        final[1] = (double)return_array[1];
    }

    final[2] = distance_between_two_points_calculating(dx, dy, (double)final[0], (double)final[1]);

    rtn = final;
    return rtn;
}

/*double* ultrasonic_rtn(int x, int y, double yaw, double angle, int* obstract_x, int* obstract_y, int ob_length, int car_x, int car_y, int width, int height)
{
    double* one_data;
    double* rtn;
    double return_values[7];
    double detect_angle_list[3] = {-7.0, 0.0, 7.0};
    double X[3] = {0.0};
    double Y[3] = {0.0};
    double distance[3] = {0.0};
    int cnt = 0;

    for (cnt = 0; cnt < 3; cnt++)
    {
        one_data = points_output(x, y, yaw, angle, detect_angle_list[cnt], obstract_x, obstract_y, ob_length, car_x, car_y, width, height);
        X[cnt] = one_data[0];
        Y[cnt] = one_data[1];
        distance[cnt] = one_data[2];
    }

    return_values[0] = min_3d(distance[0], distance[1], distance[2]);
    return_values[1] = X[0];
    return_values[2] = Y[0];
    return_values[3] = X[1];
    return_values[4] = Y[1];
    return_values[5] = X[2];
    return_values[6] = Y[2];

    rtn = return_values;
    return rtn;
}*/

double ultrasonic_rtn(int x, int y, double yaw, double angle, int* obstract_x, int* obstract_y, int ob_length, int car_x, int car_y, int width, int height)
{
    double* one_data;
    double return_values = 0.0;
    double detect_angle_list[3] = { -7.0, 0.0, 7.0 };
    double distance[3] = { 0.0 };
    int cnt = 0;

    for (cnt = 0; cnt < 3; cnt++)
    {
        one_data = points_output(x, y, yaw, angle, detect_angle_list[cnt], obstract_x, obstract_y, ob_length, car_x, car_y, width, height);
        distance[cnt] = one_data[2];
    }

    return min_3d(distance[0], distance[1], distance[2]);
}

int cresh(int* crash_x, int* crash_y, int cr_length, int ax, int ay, double negative_radian_angle, int* obstract_x, int* obstract_y, int ob_length)
{
    int cnt = 0;
    int ccnt = 0;
    int cccnt = 0;
    int length = cr_length;
    int* crx = (int*)malloc(sizeof(int) * length);
    int* cry = (int*)malloc(sizeof(int) * length);
    int minv = 0;
    int maxv = 0;

    double* rm;
    for (cnt = 0; cnt < length; cnt++)
    {
        rm = rotation_matrix((double)crash_x[cnt], (double)crash_y[cnt], (double)ax, (double)ay, negative_radian_angle);
        crx[cnt] = (int)round(rm[0]);
        cry[cnt] = (int)round(rm[1]);
    }

    for (cnt = 0; cnt < length; cnt++)
    {
        ccnt = cnt;
        minv = crx[cnt];
        maxv = crx[cnt];
        for (ccnt = cnt; ccnt < length; ccnt++)
        {
            if (cry[cnt] == cry[ccnt])
            {
                minv = xycar_min(crx[cnt], crx[ccnt]);
                maxv = xycar_max(crx[cnt], crx[ccnt]);
                break;
            }
        }
        for (cccnt = minv; cccnt <= maxv; cccnt++)
        {
            if (point_location_match(cccnt, cry[cnt], obstract_x, obstract_y, ob_length) == 1)
            {
                free(crx);
                free(cry);
                return 1;
            }
        }
    }
    free(crx);
    free(cry);
    return 0;
}

double* steering_to_wheel_ang(char vector, double steering_angle, double tread, double wheel_base, double yaw)
{
    double* rtn;
    double ud_theta[2] = { 0.0, 0.0 };
    double square = 2.0;
    double buho = 1.0;
    double theta;
    double half_tread;
    double middle_r;
    double back_whl_r, back_down_whl_r, back_up_whl_r;
    double down_r, up_r;
    double down_sin_theta, down_cos_theta, down_theta;
    double up_sin_theta, up_cos_theta, up_theta;

    if (steering_angle == (double)0.0)
    {
        rtn = ud_theta;
        return rtn;
    }

    theta = getRadians(steering_angle);

    half_tread = fabs(tread / (double)2.0);
    middle_r = wheel_base / sin(theta);
    back_whl_r = middle_r * cos(theta);
    back_down_whl_r = back_whl_r - half_tread;
    back_up_whl_r = back_whl_r + half_tread;

    down_r = sqrt(pow(back_down_whl_r, square) + pow(wheel_base, square));
    up_r = sqrt(pow(back_up_whl_r, square) + pow(wheel_base, square));

    if (vector == 'R') buho = -1.0;
    if (vector == 'L') buho = 1.0;

    down_sin_theta = asinh(buho * wheel_base / down_r);
    down_cos_theta = acosh(buho * back_down_whl_r / down_r);
    down_theta = ((down_cos_theta > down_sin_theta) ? down_cos_theta : down_sin_theta);

    up_sin_theta = asinh(buho * wheel_base / up_r);
    up_cos_theta = acosh(buho * back_up_whl_r / up_r);
    up_theta = ((up_cos_theta > up_sin_theta) ? up_cos_theta : up_sin_theta);

    ud_theta[0] = getDegrees(up_theta) + yaw;
    ud_theta[1] = getDegrees(down_theta) + yaw;

    rtn = ud_theta;
    return rtn;
}
