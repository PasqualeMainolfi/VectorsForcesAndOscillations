/*
SOHCAHTOA

S = sine
C = cosine
T = tangent
O = opposite (lato opposto in un triagolo rettangolo)
A = adjacent (lato adiacente in un triagolo rettangolo)
H = hipotenuse (ipotenusa in un triagolo rettangolo)

sin(a) = oposite divide by hypotenuse
cos(a) = adjacent divide by hypotenuse
tan(a) = opposite divide by adjacent


COORDINATE CARTESIANE E COORDINATE POLARI

a = (x, y) ---> coordinate cartesiane
a = (r, a) ---> coordinate polari [r = raggio a = angolo]

dunque:

DA CARTESIANE A POLARI:
sin(a) = SOH = y/r
cos(a) = CAH = x/r

E, DA POLARI A CARTESIANE:
x = r * cos(a)
y = r * sin(a)


...ricordiamo che:
posizione += velocità
velocità += accelerazione

velocità = velocità angolare
accelerazione = accelerazione angolare

...dato un angolo a, avremo

a += aVel
aVel += aAcc
*/


    opcode fromPolarToCartesian, 0, iiii
ia, ir, iangVel, iangAcc xin

kang init ia
kaVel init iangVel
kaAcc init iangAcc

kang += kaVel ;pos += vel
kaVel += kaAcc ;vel += acc

kx = ir * cos(kang)
ky = ir * sin(kang)

    printks("x = %f \ty = %f\n", .1, kx, ky)

    endop


/*
MOTO ARMONICO

legge oraria
x(t) = r sin(wt)

velocità
v(t) = -wr cos(wt)

accelerazione
a(t) = -(w^2)r sin(wt)

e... legando accelerazione e posizione del punto che si muove, come

x(t)/a(t) = (r cos(wt))/(-(w^2)r cos(wt)) = -1/w^2

da cui si ricava

a = -(w^2)x

ovvero l'equazione che definisce il moto armonico: IN UN MOTO ARMONICO L'ACCELERAZIONE È SEMPRE PROPORZIONALE ALLA POSIZIONE x(t) SECONDO UNA
COSTANTE DI PROPORZIONALITÀ NEGATIVA.
*/

    opcode simpleHarMotion, 0, kkk
kraggio, kperiod, kstartAngle xin

kangle init 0
i2pi = 2 * $M_PI
kw = i2pi/kperiod

kx = kraggio * cos((kangle/sr) + kstartAngle)
ky = kraggio * sin((kangle) + kstartAngle)

kaAcc = -(kw^2) * kx
kaVel = -kw * ky

kangle += kaVel
kaVel += kaAcc

    printks("x = %f \tv = %f \t a = %f\n", .01, kx, kaVel, kaAcc)

    endop


/*
SIMULAZIONE DEL PENDOLO

Fp = forza esercitata dal pendolo
Fg = forza di gravità

sin(a) = Fp/Fg
Fp = Fg * sin(a) = g * massa * sin(a)

accelerazione = F/m
acc = Fp/massa = (g * massa/massa) * sin(a) = g * sin(a)

da cui:
-g/r * sin(a) = equazione del moto del pendolo. Ad influire è solo la lunghezza del braccio del pendolo e non la massa
*/

    opcode pendulumMotion, 0, iiii
istartAngle, ilen, imassa, idamping xin

ig = 1 ;9.81

kangle init istartAngle
kaVel init 0
kaAcc init 0

//x ed y ---> vettore corpo del pendolo
kx = ilen * cos(kangle)
ky = -ilen * sin(kangle)

kWave = kx * ky

iGrav = -ig * imassa
kaAcc = ((iGrav/ilen) * sin(kangle))/imassa //equazione del moto del pendolo

kaVel += kaAcc ;accelerazione angolare
kangle += kaVel ;posizione del pendolo
kaVel *= idamping ;damping

    printks("x = %f \ty = %f \ts = %f\n", .05, kx, ky, kWave)

kaAcc *= 0

    endop

/*
MASSA - MOLLA - SMORZATORE

F = -k * x (a k maggiori molla rigida e viceversa)
x = lunghezza dopo allungamento - lunghezza a riposo della molla

Fmagnitude = sqrt(F^2)
x = Fmagnitude - lunghezza a riposo
F = normalize(F)
F = -k * x
*/

    opcode springMotion, 0, iiiii
imassa, irestLen, ipos, ik, idamp xin

kx init ipos
kv init 0
ka init 0
kSpringForce init ipos

kx += kv
kT = 2 * $M_PI * sqrt(imassa/ik)

kSpringForce = kx
kmag = sqrt(kSpringForce^2)
kallungamento = kmag - irestLen

kSpringForce = kSpringForce/kmag
kSpringForce *= (-ik * kallungamento)

    k1 = kmag * sin(2 * $M_PI * kx)

ka /= imassa
ka += kSpringForce
kv += ka
kv *= idamp

    printks("x = %f \tv = %f \ta = %f \ts = %f \tfreq = %f\n", .01, kx, kv, ka, k1, kT)

ka *= 0

    endop


/*
DOPPIO PENDOLO

m1 = massa pendolo 1
m2 = massa pendolo 2
r1 = lunghezza braccio pendolo 1
r2 = lunghezza braccio pendolo 2
g = 9.81 ---> gravità
a1 = angolo pendolo 1
a2 = angolo pendolo 2

posizione pendolo 1;
x1 = r1 * cos(a1)
y1 = -r1 * sin(a1)

posizione pendolo 2:
x2 = x1 + r2 * cos(a2)
y2 = y1 + r2 * sin(a2)

v1 = 0 ( = sqrt(g/r1))---> derivata prima velocità pendolo 2
v2 = 0 ( = sqrt(g/r2))---> derivata prima velocità pendolo 2

accNum1 = -g(2m1 + m2)sin(a1) - m2 * g * sin(a1 - 2 * a2) - 2 * sin(a1 - a2) * m2 * (v2^2 * r2 + v1^2 * r1 * cos(a1 - a2))
accDen1 = r1 * (2 * m1 + m2 - m2 * cos(2 * a1 - 2 * a2))
acc1 = accNum1/accDen1 ---> derivata seconda accelerazione pendolo 1

accNum2 = 2 * sin(a1 - a2) * (v1^2 * r1 * (m1 + m2) + g * (m1 + m2) * cos(a1) + v2^2 * r2 * m2 * cos(a1 - a2))
accDen2 = r1 * (2 * m1 + m2 - m2 * cos(2 * a1 - 2 * a2))
acc2 = accNum2/accDen2 ---> derivata seconda accelerazione pendolo 2

v1 += acc1
v2 += acc2
a1 += v1
a2 += v2
*/

    opcode doublePendulumMotion, 0, iiiiiiiik
im1, im2, ir1, ir2, ia1, ia2, ig, idamp, kdt xin

imu = 1 + im1/im2

ka1 init ia1
ka2 init ia2
kv1 init 0
kv2 init 0
kacc1, kacc2 init 0

    kx1 = ir1 * cos(ka1)
    ky1 = -ir1 * sin(ka1)
    kx2 = kx1 + ir2 * cos(ka2)
    ky2 = ky1 - ir2 * sin(ka2)

//Lagrangian
; knum_1a = ig * (sin(ka2) * cos(ka1 - ka2) - imu * sin(ka1))
; knum_1b = -(ir2 * (kv2^2) + ir1 * (kv1^2) * cos(ka1 - ka2))
; knum_1c = sin(ka1 - ka2)
; kden_1 = ir1 * (imu - (cos(ka1 - ka2))^2)
;
; knum_2a = ig * imu * (sin(ka1) * cos(ka1 - ka2) - sin(ka2))
; knum_2b = (imu * ir1 * (kv1^2) + ir2 * (kv2^2) * cos(ka1 - ka2))
; knum_2c = sin(ka1 - ka2)
; kden_2 = ir1 * (imu - (cos(ka1 - ka2))^2)
;
; kacc1 = (knum_1a + knum_1b * knum_1c)/kden_1
; kacc2 = (knum_2a + knum_2b * knum_2c)/kden_2

//https://www.myphysicslab.com/pendulum/double-pendulum-en.html
knum_1a = -ig * (2 * im1 * im2) * sin(ka1)
knum_1b = -im2 * ig * sin(ka1 - 2 * ka2)
knum_1c = -2 * sin(ka1 - ka2) * im2
knum_1d = kv2 * kv2 * ir2 + kv1 * kv1 * ir1 * cos(ka1 - ka2)
kden_1 = ir1 * (2 * im1 + im2 - im2 * cos(2 * ka1 - 2 * ka2))

knum_2a = 2 * sin(ka1 - ka2)
knum_2b = kv1 * kv1 * ir1 * (im1 + im2)
knum_2c = ig * (im1 + im2) * cos(ka1)
knum_2d = kv2 * kv2 * ir2 * im2 * cos(ka1 - ka2)
kden_2 = ir2 * (2 * im1 + im2 - im2 * cos(2 * ka1 - 2 * ka2))

kacc1 = (knum_1a + knum_1b + knum_1c * knum_1d)/kden_1
kacc2 = (knum_2a * (knum_2b + knum_2c + knum_2d))/kden_2

kv1 += (kacc1 * kdt)
kv2 += (kacc2 * kdt)
ka1 += (kv1 * kdt)
ka2 += (kv2 * kdt)
kv1 *= idamp
kv2 *= idamp

    printks("x1 = %f \ty1 = %f \tv1 = %f \ta1 = %f \tx2 = %f \ty2 = %f \tv2 = %f \ta2 = %f\n", .01, kx1, ky1, kv1, ka1, kx2, ky2, kv2, ka2)

    endop

