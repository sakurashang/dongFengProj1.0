# @Time    : 2021/9/27 下午8:40
# @Author  : WenLong
# @Email   : 47407469@qq.com
# @File    : func_test_del.py
# @Software: PyCharm
# 验证旋转角度
import math
from pyproj import Proj, transform


def calcAngOfY_Axis(vector):
    """
        计算向量与Y轴的顺时针夹角，范围0~360°
    :param vector:
    :return: 返回两个向量的夹角
    """
    # endAng=0
    vector1 = (0, 1)  # Y axis
    vector2 = vector
    AB = [0, 0, vector1[0], vector1[1]]
    CD = [0, 0, vector2[0], vector2[1]]

    def angle(v1, v2):
        # 计算v1，v2两角的0~180度角度
        dx1 = v1[2] - v1[0]
        dy1 = v1[3] - v1[1]
        dx2 = v2[2] - v2[0]
        dy2 = v2[3] - v2[1]
        angle1 = math.atan2(dy1, dx1)
        angle1 = angle1 * 180 / math.pi
        # print(angle1)
        angle2 = math.atan2(dy2, dx2)
        angle2 = angle2 * 180 / math.pi
        # print(angle2)
        if angle1 * angle2 >= 0:
            included_angle = abs(angle1 - angle2)
        else:
            included_angle = abs(angle1) + abs(angle2)
            if included_angle > 180:
                included_angle = 360 - included_angle
        return included_angle

    ang1 = angle(AB, CD)
    # 浮点数应该可以比较大小，但是比较相等要用精度判断
    # if waypoint_x - x > 0:
    #     # 在右边，0~180
    #     endAng = ang1
    # else:
    #     # 在左边，180~360
    #     endAng = 360 - ang1
    if vector2[0] < 0:
        ang1 = 360 - ang1
    # print("小车指向目标点向量与Y轴夹角=", ang1)
    return ang1

if __name__ == '__main__':
    A = (488235.0488366564, 4466139.434445432)
    B = (488092.4020280437, 4466151.642338371)
    AB_vector = (B[0] - A[0], B[1] - A[1])  # 94.89152587461189
    ang1 = calcAngOfY_Axis(AB_vector)

    if ang1 >= 180:
        ang1 = -(360-ang1)

    print(ang1)