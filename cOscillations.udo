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

kx = kraggio * sin((kangle/sr) + kstartAngle)
ky = kraggio * cos((kangle) + kstartAngle)

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

ig = 9.81

kangle init istartAngle
kaVel init 0
kaAcc init 0

//x ed y ---> vettore corpo del pendolo
kx = ilen * sin(kangle)
ky = ilen * cos(kangle)

kWave = kx * ky

iGrav = -ig * imassa
kaAcc = ((iGrav/ilen) * sin(kangle))/imassa //equazione del moto del pendolo


kangle += kaVel ;posizione del pendolo
kaVel += kaAcc ;accelerazione angolare
kaVel *= idamping ;damping

    printks("x = %f \ty = %f \ts = %f\n", .05, kx, ky, kWave)

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

    printks("x = %f \tv = %f \ta = %f \ts = %f\n", .01, kx, kv, ka, k1)

ka *= 0

    endop
