from math import pi
from math import sin
from math import cos


def zone(lastzone, a, F, B, R, L):

    yaw = a

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

            return Zone


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
