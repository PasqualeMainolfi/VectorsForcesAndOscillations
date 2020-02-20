<CsoundSynthesizer>
<CsOptions>
-n
</CsOptions>
<CsInstruments>

sr = 44100
ksmps = 1
nchnls = 2
0dbfs = 1


#include "cVectors.udo"

  instr 1 //friction and drag force application

kPos[] = createVector(0, 0)
kVel[] = createVector(0, 0)
kAcc[] = createVector(0, 0)
kForce[] = createVector(0, .1)
imassa = .1 ;massa

kFric[] = kVel
imu = .1 ;coefficiente di frizione

kDrag[] = kVel
iCr = .3

iter = 10

ki init 0
until (ki == iter) do

  kPos[] = addVector(kPos, kVel)

  kFric[] = normalizeVector(kVel)
  kFric[] = multScalVector(kFric, -imu)

  kDrag[] = normalizeVector(kVel)
  kmag = magnitude(kVel)
  kc = (kmag^2) * iCr
  kDrag[] = multScalVector(kDrag, -kc)

  kAcc[] = diviScalVector(kAcc, imassa)
  kAcc[] = addVector(kAcc, kForce + kFric + kDrag)
  kVel[] = addVector(kVel, kAcc)

  kAcc[] = multScalVector(kAcc, 0)

  printf("with force: x = %f, y = %f\n", ki + 1, kPos[0], kPos[1])

  ki += 1
od

  endin

  instr 2 //spring force

irestLen = .3
im = 1
ik = .5
idamp = .99

kPosition[] = createVector(0, .5)
kOrigin[] = createVector(0, 0)
kVel[] = createVector(0, 0)
kAcc[] = createVector(0, 0)
kSpringForce[] init 2

kallungamento init 0

ki init 0
while (ki < 1000) do

  kPosition[] = addVector(kPosition, kVel)

  kSpringForce[] = subVector(kPosition, kOrigin)
  kmag = magnitude(kSpringForce) ;lunghezza istantanea
  kallungamento = kmag - irestLen ;calcolo dell'allungamento istantaneo

  kSpringForce[] = normalizeVector(kSpringForce)
  kSpringForce[] = multScalVector(kSpringForce, -ik * kallungamento)

  kAcc[] = diviScalVector(kAcc, im)
  kAcc[] = addVector(kAcc, kSpringForce)
  kVel[] = addVector(kVel, kAcc)
  kVel[] = multScalVector(kVel, idamp)

    printf("x = %f \ty = %f\n", ki + 1, kPosition[0], kPosition[1])

  kAcc[] = multScalVector(kAcc, 0)

  ki += 1
od

  endin


  instr 3 //attrazione gravitazionale tra due corpi

iG = .0001 ;6.67 * (10^-11) //costante gravitazionle

imassMover = .3 //massa corpo in movimento
kposMover[] = createVector(.03, .5)
kvelMover[] = createVector(iG, 0)
kaccMover[] = createVector(0, 0)

imassAttractor = 1 //massa attrattore
kposAttractor[] = createVector(0, 0)
kforce[] init 2

ki init 1
while (ki > 0) do

  kforce[] = subVector(kposAttractor, kposMover)
  kforceAttractor[] = attractor(kforce, imassAttractor, imassMover, iG)

  kaccMover[] = diviScalVector(kaccMover, imassMover)
  kaccMover[] = addVector(kaccMover, kforceAttractor)
  kvelMover[] = addVector(kvelMover, kaccMover)
  kposMover[] = addVector(kposMover, kvelMover)

    printf("x = %f \ty = %f\n", ki + 1, kposMover[0], kposMover[1])

  kaccMover[] = multScalVector(kaccMover, 0)

  ki += 1
od

  endin


</CsInstruments>
<CsScore>

//i 1 0 1
i 2 0 5
//i 3 0 5
</CsScore>
</CsoundSynthesizer>

