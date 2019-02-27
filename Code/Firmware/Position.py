# TODO: Write route2 function -> IAN code
import coordinate as cd
from math import pi
from math import sin
from math import cos

# values imported and stored as the variables below
# ==================================================
#yaw = -157.335
#front = 137
##back = 8000
#right = 8000
#left = 123




def ZoneIdentification(lastzone, yaw, F, B, R, L):

    a = yaw

    b = (90+a)*pi/180
    c = -a*pi/180
    d = (90-a)*pi/180
    e = (180-a)*pi/180
    f = (a-90)*pi/180
    g = (-a-90)*pi/180
    h = (180+a)*pi/180


    if lastzone == 1:

        if yaw >= -180 and yaw < -90:
            Zone = 1

            return(Zone)


        if yaw >= -90 and yaw < 0:
            if ((B*cos(a) < 150) and (F*cos(a) >150) and (L*cos(a) > 150)) or ((R*cos(b) < 150) and (B*sin(c) > 150) and (L*cos(b) > 150)) or ((B*cos(a)<150) and (R*cos(b)<150) and (L*cos(b)>150)):
                Zone = 1
            else:
                Zone = 2

            return(Zone)


        if yaw >= 0 and yaw < 90:
            if ((L*sin(d)<150) and (F*cos(a)<150) and (B*cos(a)<150)) or ((F*sin(a)<150) and (L*cos(d)<150) and (R*cos(d)>150)) or ((F*sin(a)<150) and (L*sin(d)<150) and (R*cos(d)>150)):
                Zone = 4
            else:
                if ((B*cos(a)<150) and (F*cos(a)>150) and (L*sin(d)>150)) or ((R*cos(d)>150) and (B*sin(a)<150) and (L*cos(d)<150)) or ((B*cos(a)<150) and (L*cos(d)<150) and (R*cos(a)<150)):
                    Zone = 1
                else:
                    Zone = 2

            return(Zone)


        if yaw >= 90 and yaw < 180:
            if ((F*sin(e)>150) and (L*cos(f)<150) and (B*cos(e)<150)) or ((R*sin(f)>150) and (L*cos(f)<150) and (F*cos(e)<150)) or ((F*cos(e)<150) and (L*cos(f)<150) and (B*sin(e)<150)):
                Zone = 1
            else:
                Zone = 4

            return(Zone)

    if lastzone == 2:

        if yaw >= -180 and yaw < -90:
            if ((L*cos(g)>150) and (R*cos(g)<150) and (F*cos(g)<150)) or ((L*sin(g)<150) and (F*cos(h)<150) and (R*sin(g)>150)) or ((F*cos(h)<150) and (R*cos(g)<150) and (B*cos(h)>150)):
                Zone = 1
            else:
                Zone = 2

            return(Zone)


        if yaw >= -90 and yaw < 0:
            Zone = 2

            return(Zone)


        if yaw >= 0 and yaw < 90:
            if ((R*cos(d)<150) and (L*cos(a)>150) and (F*cos(a)>150)) or ((F*sin(a)>150) and (B*sin(a)<150) and (R*cos(d)<150)) or ((L*sin(d)>150) and (B*sin(a)>150) and (F*cos(a)<150)):
                Zone = 2
            else:
                Zone = 3

            return(Zone)


        if yaw >= 90 and yaw < 180:
            if ((F*cos(f)<150) and (R*cos(f)<150) and (L*cos(e)>150)) or ((R*sin(f)<150) and (B*cos(e)<150) and (L*sin(f)>150)) or ((F*cos(f)<150) and (R*sin(f)<150) and (L*cos(f)>150)):
                Zone = 3
            else:
                if ((F*sin(e)>150) and (L*cos(f)<150) and (B*cos(e)<150)) or ((R*sin(f)>150) and (L*cos(f)<150) and (F*cos(e)<150)) or ((F*cos(e)<150) and (L*cos(f)<150) and (B*sin(e)<150)):
                    Zone = 1
                else:
                    Zone = 2

            return(Zone)

    if lastzone == 3:

        if yaw >= -180 and yaw < -90:
            if ((R*cos(g)<150) and (B*sin(h)<150) and (L*cos(g)>150)) or ((F*cos(h)<150) and (R*sin(g)<150) and (L*sin(g)>150)) or ((R*cos(g)<150) and (F*cos(h)<150) and (L*sin(g)>150)):
                Zone = 4
            else:
                if ((F*cos(g)<150) and (B*sin(h)>150) and (L*cos(g)<150)) or ((F*cos(h)>150) and (R*sin(g)>150) and (L*sin(g)<150)) or ((R*cos(g)>150) and (F*cos(g)<150) and (L*sin(g)<150)):
                    Zone = 2
                else:
                    Zone = 3

            return(Zone)


        if yaw >= -90 and yaw < 0:
            if ((R*sin(b)>150) and (L*cos(b)<150) and (F*cos(a)>150)) or ((L*cos(b)<150) and (B*sin(c)<150) and (R*cos(b)>150)) or ((F*cos(a)<150) and (L*cos(b)<150) and (R*sin(b)>150)):
                Zone = 3
            else:
                zone = 2

            return(Zone)


        if yaw >= 0 and yaw < 90:
            Zone = 3

            return(Zone)


        if yaw >= 90 and yaw < 180:
            if ((F*cos(f)<150) and (R*cos(f)<150) and (L*cos(e)>150)) or ((R*sin(f)<150) and (B*cos(e)<150) and (L*sin(f)>150)) or ((F*cos(f)<150) and (R*sin(f)<150) and (L*cos(f)>150)):
                Zone = 3
            else:
                Zone = 4

            return(Zone)

    if lastzone == 4:

        if yaw >= -180 and yaw < -90:
            if ((L*cos(g)>150) and (R*cos(g)<150) and (F*cos(g)<150)) or ((L*sin(g)<150) and (F*cos(h)<150) and (R*sin(g)>150)) or ((F*cos(h)<150) and (R*cos(g)<150) and (B*cos(h)>150)):
                Zone = 1
            else:
                Zone = 2

            return(Zone)


        if yaw >= -90 and yaw < 0:
            if ((B*cos(a) < 150) and (F*cos(a) >150) and (L*cos(a) > 150)) or ((R*cos(b) < 150) and (B*sin(c) > 150) and (L*cos(b) > 150)) or ((B*cos(a)<150) and (R*cos(b)<150) and (L*cos(b)>150)):
                Zone = 1
            else:
                if ((R*sin(b)>150) and (L*cos(b)<150) and (F*cos(a)>150)) or ((L*cos(b)<150) and (B*sin(c)<150) and (R*cos(b)>150)) or ((F*cos(a)<150) and (L*cos(b)<150) and (R*sin(b)>150)):
                    Zone = 3
                else:
                    Zone = 4

            return(Zone)


        if yaw >= 0 and yaw < 90:
            if ((L*sin(d)<150) and (F*cos(a)<150) and (B*cos(a)<150)) or ((F*sin(a)<150) and (L*cos(d)<150) and (R*cos(d)>150)) or ((F*sin(a)<150) and (L*sin(d)<150) and (R*cos(d)>150)):
                Zone = 4
            else:
                Zone = 3

            return(Zone)


        if yaw >= 90 and yaw < 180:
            Zone = 4

            return(Zone)



def find_coordinate(yaw, front, back, right, left, zone):

    if yaw >= -180 and yaw < -90:
        coordinate = cd.route1(yaw, front, back, right, left, zone)

    if yaw >= -90 and yaw < 0:
        coordinate = cd.route2(yaw, front, back, right, left, zone)

    if yaw >= 0 and yaw < 90:
        coordinate = cd.route3(yaw, front, back, right, left, zone)

    if yaw >= 90 and yaw < 180:
        coordinate = cd.route4(yaw, front, back, right, left, zone)

    #print(coordinate)
    return coordinate

def best_route(COMM, org, ang_org):
    """
    Returns 1 for forwards 0 for backwards.
    Returns 1 for clockwise and 0 for anticlockwise.
    """

    deltaX = (COMM[0] - org[0])
    deltaY = (COMM[1] - org[1])

    dist = ((deltaX)**2 + (deltaY)**2)**0.5
    T = dist/v

    if deltaY == 0:
        if deltaX == 0:
            ang_desired = 0

        elif deltaX > 0:
            ang_desired = 90

        elif deltaX < 0:
            ang_desired = 270


    elif deltaX == 0:
        if deltaY > 0:
            ang_desired = 0

        elif deltaY < 0:
            ang_desired = 180


    else:
        if deltaX > 0 and deltaY > 0:
            n = 0

        elif deltaX > 0 and deltaY < 0:
            n = -180

        elif deltaX < 0 and deltaY < 0:
            n = 180

        elif deltaX < 0 and deltaY > 0:
            n = -360

        ang_desired = abs(abs(atan(deltaX / deltaY) * (180/pi)) + n)


    ang_mov = ang_desired - ang_org


    if ang_desired < ang_org:
        ang_mov = ang_mov + 360

    else:
        ang_mov = abs(ang_mov)



    if ang_mov <= 90:
        #clockwise_ang(ang_mov)
        #forward()
        rot = 1
        direct = 1


    elif ang_mov > 90 and ang_mov <= 180:

        if ang_desired >= 180 and ang_desired < 360:
            ang_desired = ang_desired - 180

        elif ang_desired < 180 and ang_desired >= 0:
            ang_desired = ang_desired + 180

        ang_mov = abs(180 - ang_mov)

        #anti_clockwise_ang(ang_mov)
        #back()
        rot = 0
        direct = 0


    elif ang_mov > 180 and ang_mov <= 270:

        if ang_desired >= 180 and ang_desired < 360:
            ang_desired = ang_desired - 180

        elif ang_desired < 180 and ang_desired >= 0:
            ang_desired = ang_desired + 180

        ang_mov = abs(180 - ang_mov)

        #clockwise_ang(ang_mov)
        #back()
        rot = 1
        direct = 0


    elif ang_mov > 270:

        ang_mov = 360 - ang_mov

        #anti_clockwise(ang_mov)
        #forward()
        rot = 0
        direct = 1

    return (rot, ang_mov), (direct, dist), ang_desired
