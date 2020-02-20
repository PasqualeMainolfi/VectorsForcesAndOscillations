/*
Force Application

ricordiamo che:

posizione += velocità
    x += v

velocità += accelerazione
    v += a

accelerazione = forza/massa
    a = F/m

forza F = massa * accelerazione
    accelerazione += forza
        a += F

forza di gravità Fg = g * massa
g = (GM/r^3)*r^
g = 9.81 m/s^2
G = 6.67 * 10^-11 Nm^2/kg^2 ---> costante gravitazionale


frizione Ff = -1 * mu * |N| * v^
    con direzione = -1 * v^
    con manitudine = mu * |N|
        |N| = normal force
        nu = coefficiente di frizione


resistenza Fr = -1/2 * ro * |v|^2 * A * Cr * v^
    con direzione = -1/2 * v^
    con magnitudine = Cr * |v|^2
        ro = densità
        v^ = vettore velocità unitario
        |v|^2 = modulo del vettore velocità al quadrato
        A = area
        Cr = costante di resistenza


massa molla Fm = -k * x
x = lunghezza dopo allungamento - lunghezza a riposo della molla

Fmagnitude = sqrt(Fm^2)
x = Fmagnitude - lunghezza a riposo
Fm = normalize(Fm)
Fm = -k * x

attrazione gravitazionale Fg = (G * m1 * m2/d^2) * r^
G = costante gravitazionale
m1 = massa corpo 1 (in movimento)
m2 = massa corpo 2 (attrattore)
d = distanza tra i due corpi
r^ = vettore unitario raggio

Fg = posA - posB
d = magnitude(Fg)
Fg = normalize(Fg) ---> direzione della forza

mag = G * m1 * m2/d^2 ---> magnitudine della forza

Fg = Fg * mag ---> applicazione direzione e magnitudine
*/

    opcode createVector, k[], kk
kx, ky xin

kV[] init 2
kV[0] = kx
kV[1] = ky

xout(kV)
    endop

    opcode addVector, k[], k[]k[]
kV1[], kV2[] xin
kV[] = kV1 + kV2
xout(kV)
    endop

    opcode subVector, k[], k[]k[]
kV1[], kV2[] xin
kV[] = kV1 - kV2
xout(kV)
    endop

    opcode multVector, k, k[]k[]
kV1[], kV2[] xin
kmult = (kV1[0] * kV2[0]) + (kV1[1] * kV2[1])
xout(kmult)
    endop

    opcode diviVector, k, k[]k[]
kV1[], kV2[] xin
kdiv = (kV1[0]/kV2[0]) + (kV1[1]/kV2[1])
xout(kdiv)
    endop

    opcode multScalVector, k[], k[]k
kV1[], kreal xin
kV[] = kV1 * kreal
xout(kV)
    endop

    opcode diviScalVector, k[], k[]k
kV1[], kreal xin
kV[] = kV1/kreal
xout(kV)
    endop

    opcode normalizeVector, k[], k[]
kV1[] xin

kx = kV1[0]
ky = kV1[1]

kNorma = sqrt((kx^2) + (ky^2))
kV[] = kV1/kNorma

xout(kV)
    endop

    opcode magnitude, k, k[]
kV1[] xin

kx = kV1[0]
ky = kV1[1]

kmag = sqrt((kx^2) + (ky^2))

xout(kmag)
    endop

    opcode friction, k[], k[]kk ;calcolo vettore forza frizione
kV1[], kmu, kNormaleForce xin

//kV1[] = vettore velocità in entrata

kmagFric = kmu * kNormaleForce
kFric[] = normalizeVector(kV1)
kFric[] = multScalVector(kFric, -kmagFric)

xout(kFric)
    endop

    opcode drag, k[], k[]kkk ;calcolo vettore forza resistenza dei materiali
kV1[], krho, kA, kCd xin

//kV1[] = vettore velocità in entrata

kmag = magnitude(kV1)
kmag = (kmag^2) * kCd
kDrag[] = normalizeVector(kV1)
kDrag[] = multScalVector(kDrag, -kmag)

xout(kDrag)
    endop


     opcode spring, k[], k[]k[]ii ;simulazione forza molla
kV1[], kO[], irestLen, ik xin

/*
kV1[] = vettore posizione massa attaccata alla molla
kO[] = vettore origini, posizione in cui è attaccata la molla
*/

kSpringForce[] = subVector(kV1, kO) ;direzione della forza

kmagSpring = magnitude(kSpringForce) ;magnitudine per il calcolo dell'allungamento
kallungamento = kmagSpring - irestLen ;calcolo allungamento

kSpringForce[] = normalizeVector(kSpringForce) ;normalizzazione
kSpringForce[] = multScalVector(kSpringForce, -ik * kallungamento) ; -k * x

xout(kSpringForce)
    endop

    opcode attractor, k[], k[]iii ;attrazione gravitazionale
kforceAttractor[], imassAttractor, imassMover, iG xin

/*
kposAttractor[] = posizione dell'attrattore
imassAttractor = massa dell'attrattore
kposMover[] = vettore posizione mover oggetto in movimento
imassMover = massa dell'oggetto in movimento
iG = costante gravitazionale
*/

kdistance = magnitude(kforceAttractor) //magnitudine della forza
kdistance = 1/kdistance

kforceAttractor[] = normalizeVector(kforceAttractor) //direzione della forza

kc = (iG * imassMover * imassAttractor)/(kdistance^2) //forza
kforceAttractor[] = multScalVector(kforceAttractor, kc) //direzione moltiplicato magnitudine

    ;printks("distance = %f\n", .01, kdistance)

xout(kforceAttractor)
    endop
