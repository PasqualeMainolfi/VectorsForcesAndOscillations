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

  instr 1

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
  kAcc[] = diviScalVector(kAcc, imassa)
  kAcc[] = addVector(kAcc, kForce + kFric + kDrag)
  kVel[] = addVector(kVel, kAcc)

  kFric[] = normalizeVector(kVel)
  kFric[] = multScalVector(kFric, -imu)

  kDrag[] = normalizeVector(kVel)
  kmag = magnitude(kVel)
  kc = (kmag^2) * iCr
  kDrag[] = multScalVector(kDrag, -kc)

  kAcc[] = multScalVector(kAcc, 0)

  printf("with force: x = %f, y = %f\n", ki + 1, kPos[0], kPos[1])

  ki += 1
od

  endin


  instr 2 //attrazione gravitazionale tra due corpi

imassMover = .7 //massa corpo in movimento
kposMover[] = createVector(.3, .005)
kvelMover[] = createVector(0, 0)
kaccMover[] = createVector(0, 0)

imassAttractor = 2.1 //massa attrattore
kposAttractor[] = createVector(0, 0)
kforceAttractor[] = kposMover


ki init 1
while (ki > 0) do

  kforceAttractor[] = attractor(kposAttractor, imassAttractor, kposMover, imassMover)

  kposMover[] = addVector(kposMover, kvelMover)
  kaccMover[] = diviScalVector(kaccMover, imassMover)
  kaccMover[] = addVector(kaccMover, kforceAttractor)
  kvelMover[] = addVector(kvelMover, kaccMover)

    printf("x = %f \ty = %f\n", ki + 1, kposMover[0], kposMover[1])

  kaccMover[] = multScalVector(kaccMover, 0)

  ki += 1
od

  endin


</CsInstruments>
<CsScore>

//i 1 0 1
i 2 0 5
</CsScore>
</CsoundSynthesizer>
