<CsoundSynthesizer>
<CsOptions>
-n ;-o dac
</CsOptions>
<CsInstruments>

sr = 44100
ksmps = 1
nchnls = 2
0dbfs = 1


#include "cOscillations.udo"
#include "cVectors.udo"


  instr 1 //da coordinate polari a coordinate cartesiane

ia = 0
ir = 1
iangVel = 0
iangAcc = .01

fromPolarToCartesian(ia, ir, iangVel, iangAcc)

  endin


  instr 2 //moto armonico semplice


kamp = 1 ;ampiezza
kT = 1/4 ;periodo
kstartAngle = 0 ;angolo di partenza

simpleHarMotion(kamp, kT, kstartAngle)

  endin


  instr 3 //simulazione pendolo


istartAngle = $M_PI/4
ibraccioPendolo = 1
imassa = 3
idamping = .93

pendulumMotion(istartAngle, ibraccioPendolo, imassa, idamping)

  endin

  instr 4 //massa - molla - smorzatore

irestLen = 0 ;equilibrio stabile del sistema. Lunghezza a riposo della molla (solo riferimento qui)
imassa = 1.2
ik = .01
idamp = .93

kOrigin[] = createVector(0, 0)
kposMassa[] = createVector(0, .3)
kVel[] = createVector(0, 0)
kAcc[] = createVector(0, 0)
kSpringForce[] = kposMassa


ki init 0
while (ki < 500) do

  kposMassa[] = addVector(kposMassa, kVel)

  kSpringForce[] = subVector(kposMassa, kOrigin)
  kmagSpring = magnitude(kSpringForce) //magnitudine
  kallungamento = kmagSpring - irestLen //differneza di lunghezza

  kSpringForce[] = normalizeVector(kSpringForce)
  kSpringForce[] = multScalVector(kSpringForce, -ik * kallungamento) //F = -k * x

  kAcc[] = diviScalVector(kAcc, imassa)
  kAcc[] = addVector(kAcc, kSpringForce)
  kVel[] = addVector(kVel, kAcc)
  kVel[] = multScalVector(kVel, idamp)

    printf("x = %f \ty = %f\n", ki + 1, kposMassa[0], kposMassa[1])

  kAcc[] = multScalVector(kAcc, 0)

    ki += 1
od
  endin


  instr 5 //spring motion

irestLen = 0
imassa = 1
ik = .05
ipos = .5
idamp = 1 - (10^-4)

springMotion(imassa, irestLen, ipos, ik, idamp)

  endin


  instr 6 //doppio pendolo

imassa1 = .1
imassa2 = .1
ibraccio1 = .5
ibraccio2 = .5
iangle1 = $M_PI/2
iangle2 = $M_PI/2
ig = 1
idamp = 1
kdt = .01

doublePendulumMotion(imassa1, imassa2, ibraccio1, ibraccio2, iangle1, iangle2, ig, idamp, kdt)

  endin


</CsInstruments>
<CsScore>

//i 1 0 3
//i 2 0 3
//i 3 0 10
//i 4 0 10
//i 5 0 5
i 6 0 10

</CsScore>
</CsoundSynthesizer>
